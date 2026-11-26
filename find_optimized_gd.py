"""Optimize grouped PWM with a sensor response matrix measured once at startup."""

import argparse
import csv
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from find_optimized import (
    GROUP_COUNT,
    PWM_GROUPS as DEFAULT_PWM_GROUPS,
    PWM_STEP,
    SENSOR_IDS,
    Measurement,
    armed_teensy,
    deviation_pct,
    expand_pwm,
    load_f2_calibration,
    quantize,
    result_data,
    validate_pwm_groups,
    write_result,
)


# Use the LP script's symmetry groups by default. To override, enter six tuples
# of four channel numbers here, with each channel from 1 to 24 used once.
PWM_GROUPS = DEFAULT_PWM_GROUPS
INITIAL_PWM_PCT = 50.0
MAX_PWM_PCT = 50.0
BRIGHTNESS_WEIGHT = 0.5      # RMS F2 difference traded for one count of mean F2.
UNIFORMITY_PENALTY_START_PCT = 5.0
UNIFORMITY_PENALTY_WEIGHT = 25.0  # Loss points per percentage point above target.
STEP_L2_PCT = 3.0            # Initial Euclidean length of the six-group PWM step (%p).
STEP_DECAY_PATIENCE = 2      # Halve the step after this many consecutive iterations without a new best.
NO_IMPROVEMENT_PATIENCE = 5  # Stop after this many consecutive iterations without a new best.
MAX_ITERATIONS = 100
CALIBRATION_REPEATS = 3
SEARCH_REPEATS = 3
VERIFY_REPEATS = 5
SETTLE_SECONDS = 0.3
SERIAL_PORT = "COM3"


def validate_settings():
    validate_pwm_groups(PWM_GROUPS)
    if not PWM_STEP <= INITIAL_PWM_PCT <= MAX_PWM_PCT <= 100:
        raise ValueError("PWM 설정은 0.1 <= INITIAL_PWM_PCT <= MAX_PWM_PCT <= 100이어야 합니다.")
    if quantize((INITIAL_PWM_PCT, MAX_PWM_PCT)) != (INITIAL_PWM_PCT, MAX_PWM_PCT):
        raise ValueError("초기 PWM과 PWM 상한은 소수점 한 자리까지 설정하세요.")
    if not math.isfinite(BRIGHTNESS_WEIGHT) or BRIGHTNESS_WEIGHT < 0:
        raise ValueError("BRIGHTNESS_WEIGHT는 0 이상의 유한한 값이어야 합니다.")
    if not 0 <= UNIFORMITY_PENALTY_START_PCT < 100:
        raise ValueError("UNIFORMITY_PENALTY_START_PCT는 0 이상 100 미만이어야 합니다.")
    if not math.isfinite(UNIFORMITY_PENALTY_WEIGHT) or UNIFORMITY_PENALTY_WEIGHT <= 0:
        raise ValueError("UNIFORMITY_PENALTY_WEIGHT는 0보다 큰 유한한 값이어야 합니다.")
    if not math.isfinite(STEP_L2_PCT) or STEP_L2_PCT < PWM_STEP:
        raise ValueError(f"STEP_L2_PCT는 {PWM_STEP:g} 이상의 유한한 값이어야 합니다.")
    if not math.isfinite(SETTLE_SECONDS) or SETTLE_SECONDS < 0:
        raise ValueError("SETTLE_SECONDS는 0 이상의 유한한 값이어야 합니다.")
    counts = (
        MAX_ITERATIONS, STEP_DECAY_PATIENCE, NO_IMPROVEMENT_PATIENCE, CALIBRATION_REPEATS,
        SEARCH_REPEATS, VERIFY_REPEATS,
    )
    if any(type(count) is not int or count < 1 for count in counts):
        raise ValueError("반복 횟수는 1 이상의 정수여야 합니다.")


def measure_groups(teensy, pwm, repeats):
    teensy.set_channel_pwm(expand_pwm(pwm, groups=PWM_GROUPS), SETTLE_SECONDS)
    samples = teensy.read_f2_samples(repeats)
    if samples is None:
        print(f"PWM {pwm}  센서 포화", flush=True)
        return None
    measurement = Measurement(pwm, samples)
    print(
        f"PWM {pwm}  F2 {measurement.f2.tolist()}  "
        f"평균 {measurement.mean:.1f}  최대 편차 {measurement.deviation:.2f}%",
        flush=True,
    )
    return measurement


def calibrate_once(teensy):
    starting_pwm = (INITIAL_PWM_PCT,) * GROUP_COUNT
    baseline = measure_groups(teensy, starting_pwm, CALIBRATION_REPEATS)
    if baseline is None or baseline.mean <= 0:
        raise RuntimeError("초기 밝기를 측정할 수 없습니다. 초기 PWM과 센서 적분시간을 확인하세요.")

    baseline_f2 = baseline.f2
    starting_channels = expand_pwm(starting_pwm, groups=PWM_GROUPS)
    group_slopes = np.empty((len(SENSOR_IDS), GROUP_COUNT))
    for index, channels in enumerate(PWM_GROUPS):
        probe_pwm = list(starting_pwm)
        probe_pwm[index] = 0.0
        off_measurement = measure_groups(teensy, tuple(probe_pwm), CALIBRATION_REPEATS)
        teensy.set_channel_pwm(starting_channels, SETTLE_SECONDS)
        if off_measurement is None:
            raise RuntimeError(f"그룹 {index + 1}의 반응 측정 중 센서가 포화되었습니다.")
        group_slopes[:, index] = (baseline_f2 - off_measurement.f2) / INITIAL_PWM_PCT
        print(
            f"그룹 {index + 1} {channels} 반응 (F2 counts/%p) {group_slopes[:, index]}",
            flush=True,
        )
    return baseline, group_slopes


def uniformity_penalty(f2):
    excess = max(0.0, deviation_pct(f2) - UNIFORMITY_PENALTY_START_PCT)
    return UNIFORMITY_PENALTY_WEIGHT * excess


def loss(f2, reference_mean):
    rms = f2.std()
    brightness_gain = f2.mean() - reference_mean
    base_loss = 100 * (rms - BRIGHTNESS_WEIGHT * brightness_gain) / reference_mean
    return float(base_loss + uniformity_penalty(f2))


def loss_gradient(f2, group_slopes, reference_mean):
    """Differentiate the loss using the fixed group response matrix."""
    mean = f2.mean()
    deviations = f2 - mean
    rms = f2.std()
    mean_slope = group_slopes.mean(axis=0)
    gradient = -BRIGHTNESS_WEIGHT * mean_slope
    if rms > 0:
        gradient += deviations @ (group_slopes - mean_slope) / (deviations.size * rms)
    gradient *= 100 / reference_mean

    abs_deviations = np.abs(deviations)
    worst = abs_deviations.max()
    if 100 * worst / mean > UNIFORMITY_PENALTY_START_PCT:
        # At a tie, average the active sensors' subgradients.
        active = abs_deviations == worst
        signed_slopes = group_slopes[active] - (f2[active, None] / mean) * mean_slope
        abs_slopes = np.sign(deviations[active, None]) * signed_slopes
        gradient += UNIFORMITY_PENALTY_WEIGHT * (100 / mean) * abs_slopes.mean(axis=0)
    return gradient


def next_pwm(pwm, gradient, step_l2_pct):
    """Take one step of the given L2 length before applying PWM limits."""
    current = np.asarray(pwm)
    direction = -gradient.copy()
    direction[(current <= 0) & (direction < 0)] = 0
    direction[(current >= MAX_PWM_PCT) & (direction > 0)] = 0
    magnitude = np.linalg.norm(direction)
    if magnitude <= 1e-12:
        return pwm
    step = (direction / magnitude) * step_l2_pct
    return quantize(np.clip(current + step, 0, MAX_PWM_PCT))


def optimize(teensy, iteration_file):
    writer = csv.writer(iteration_file)
    writer.writerow([
        "iteration",
        *(f"sensor_{sensor}_f2" for sensor in SENSOR_IDS),
        *(f"ch{channel:02d}_pwm_pct" for channel in range(1, 25)),
    ])
    iteration_file.flush()

    def log_iteration(iteration, pwm, f2):
        channels = expand_pwm(pwm, groups=PWM_GROUPS)
        writer.writerow([iteration, *f2, *channels])
        iteration_file.flush()

    print("초기 그룹별 반응 측정", flush=True)
    baseline, group_slopes = calibrate_once(teensy)
    reference_mean = baseline.mean
    # Refresh the starting reading after calibration. Keep the measured slopes fixed.
    current = measure_groups(teensy, baseline.pwm, SEARCH_REPEATS)
    if current is None or current.mean <= 0:
        raise RuntimeError("그룹 반응 측정 후 초기 PWM의 센서값을 확인할 수 없습니다.")
    current_loss = loss(current.f2, reference_mean)
    best = current
    best_loss = current_loss
    best_iteration = 0
    log_iteration(0, current.pwm, current.f2.tolist())
    accepted_steps = 0
    iterations_without_improvement = 0
    step_l2_pct = STEP_L2_PCT
    stop_reason = "iteration_limit"

    for iteration in range(1, MAX_ITERATIONS + 1):
        gradient = loss_gradient(current.f2, group_slopes, reference_mean)
        print(
            f"반복 {iteration}/{MAX_ITERATIONS}  손실 {current_loss:.4f} · "
            f"스텝 길이 {step_l2_pct:g}%p",
            flush=True,
        )
        trial = next_pwm(current.pwm, gradient, step_l2_pct)
        if trial == current.pwm or not any(trial):
            log_iteration(iteration, current.pwm, current.f2.tolist())
            stop_reason = "no_feasible_step"
            break

        result = measure_groups(teensy, trial, SEARCH_REPEATS)
        f2 = result.f2.tolist() if result is not None else [None] * len(SENSOR_IDS)
        log_iteration(iteration, trial, f2)
        if result is None or result.mean <= 0:
            stop_reason = "invalid_measurement"
            break

        trial_loss = loss(result.f2, reference_mean)
        loss_change = trial_loss - current_loss
        current = result
        current_loss = trial_loss
        accepted_steps += 1
        if current_loss < best_loss:
            best = current
            best_loss = current_loss
            best_iteration = iteration
            iterations_without_improvement = 0
        else:
            iterations_without_improvement += 1
        print(f"손실 {current_loss:.4f} · 변화량 {loss_change:+.4f}", flush=True)
        if iterations_without_improvement >= NO_IMPROVEMENT_PATIENCE:
            stop_reason = "no_new_best"
            break
        if (
            iterations_without_improvement
            and iterations_without_improvement % STEP_DECAY_PATIENCE == 0
            and step_l2_pct > PWM_STEP
            and iteration < MAX_ITERATIONS
        ):
            step_l2_pct = max(PWM_STEP, step_l2_pct / 2)
            print(f"다음 반복 스텝 길이 {step_l2_pct:g}%p", flush=True)

    print(f"최저 손실 PWM 재측정 (반복 {best_iteration}, {VERIFY_REPEATS}회 측정)", flush=True)
    final = measure_groups(teensy, best.pwm, VERIFY_REPEATS)
    if final is not None and final.mean <= 0:
        final = None
    data = result_data(final, final is not None, groups=PWM_GROUPS)
    data.update(
        method="fixed_response_gradient_descent",
        initial_pwm_pct=INITIAL_PWM_PCT,
        initial_f2_median=baseline.f2.tolist(),
        brightness_weight=BRIGHTNESS_WEIGHT,
        uniformity_penalty_start_pct=UNIFORMITY_PENALTY_START_PCT,
        uniformity_penalty_weight=UNIFORMITY_PENALTY_WEIGHT,
        reference_mean_f2=reference_mean,
        group_response_f2_per_pwm_pct=group_slopes.tolist(),
        step_l2_pct=STEP_L2_PCT,
        final_step_l2_pct=step_l2_pct,
        iterations=iteration,
        best_iteration=best_iteration,
        best_search_loss=best_loss,
        accepted_steps=accepted_steps,
        step_decay_patience=STEP_DECAY_PATIENCE,
        no_improvement_patience=NO_IMPROVEMENT_PATIENCE,
        stop_reason=stop_reason,
    )
    if final is not None:
        f2 = final.f2
        channel_pwm = data["channel_pwm_pct"]
        max_pwm = max(channel_pwm)
        data.update(
            optimized_pwm_pct=[round(100 * duty / max_pwm, 3) for duty in channel_pwm],
            final_loss=loss(f2, reference_mean),
            uniformity_penalty=uniformity_penalty(f2),
            sensor_rms_difference_f2=float(f2.std()),
            sensor_max_minus_min_f2=float(np.ptp(f2)),
        )
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=SERIAL_PORT, help="Teensy 시리얼 포트")
    parser.add_argument("--output", type=Path, help="새 결과 JSON 파일 경로")
    parser.add_argument("--calibration", type=Path, help="센서 보정 JSON (기본값은 최신 파일)")
    args = parser.parse_args()
    output = args.output or Path(f"optimized_pwm_gd_{datetime.now():%Y%m%d_%H%M%S_%f}.json")
    iteration_path = output.with_name(f"{output.stem}_iterations.csv")

    try:
        validate_settings()
        calibration = load_f2_calibration(args.calibration)
        for path in (output, iteration_path):
            if path.exists() or not path.parent.is_dir():
                raise ValueError(f"기존 파일과 겹치지 않는 결과 경로를 지정하세요. {path}")
        print(f"F2 보정 파일 {calibration.path}", flush=True)
        with iteration_path.open("x", encoding="utf-8", newline="") as iteration_file:
            print(f"반복 로그 {iteration_path.resolve()}", flush=True)
            with armed_teensy(args.port, calibration=calibration) as teensy:
                data = optimize(teensy, iteration_file)
        data["iteration_log_csv"] = iteration_path.name
        write_result(output, data, calibration=calibration)
    except KeyboardInterrupt:
        print("사용자가 탐색을 중단했습니다.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"최적화 중단. {error}", file=sys.stderr)
        return 1

    if data["status"] == "verified":
        print(
            f"검증 완료. 손실 {data['final_loss']:.4f} · 평균 F2 {data['mean_f2']:.1f} · "
            f"최대 편차 {data['max_deviation_pct']:.2f}% · 균일도 페널티 {data['uniformity_penalty']:.2f}"
        )
        print(f"CH1~24 PWM (%) {data['channel_pwm_pct']}")
    else:
        print("최종 PWM의 유효한 센서값을 확인하지 못했습니다.")
    print(f"결과 파일 {output.resolve()}")
    return 0 if data["status"] == "verified" else 2


if __name__ == "__main__":
    sys.exit(main())

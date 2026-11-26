"""Find the brightest uniform six-group PWM setting using Teensy measurements."""

import argparse
import itertools
import json
import math
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from serial import Serial, SerialException


# Replace these example groups with the actual symmetry groups. Use CH1–CH24 once each.
PWM_GROUPS = (
    (1, 6, 19, 24),
    (2, 5, 20, 23),
    (3, 4, 21, 22),
    (7, 12, 13, 18),
    (8, 11, 14, 17),
    (9, 10, 15, 16),
)
UNIFORMITY_TOLERANCE_PCT = 5.0  # Maximum deviation of each sensor from the mean (±%).
SERIAL_PORT = "COM3"
MAX_PWM_PCT = 50.0
CALIBRATION_PWM_PCT = 20.0
SETTLE_SECONDS = 0.3
SEARCH_REPEATS = 3
VERIFY_REPEATS = 5

GROUP_COUNT = 6
SENSOR_IDS = (0, 1, 2, 3)
PWM_STEP = 0.1
SERIAL_TIMEOUT_SECONDS = 12.0


@dataclass(frozen=True)
class F2Calibration:
    path: Path
    dark_offsets: tuple[float, ...]
    correction_factors: tuple[float, ...]

    def correct(self, raw_f2):
        corrected = tuple(
            (raw - dark) * factor
            for raw, dark, factor in zip(
                raw_f2, self.dark_offsets, self.correction_factors, strict=True
            )
        )
        if not all(map(math.isfinite, corrected)):
            raise ValueError("센서 보정 F2가 유한한 값이어야 합니다.")
        return corrected

    def metadata(self):
        return {
            "file": str(self.path),
            "f2_dark_offsets": self.dark_offsets,
            "f2_correction_factors": self.correction_factors,
        }


def load_f2_calibration(path=None):
    if path is None:
        for directory in dict.fromkeys((Path.cwd(), Path(__file__).resolve().parent)):
            candidates = [
                file for file in directory.glob("sensor_calibration_*.json") if file.is_file()
            ]
            if candidates:
                path = max(candidates, key=lambda file: (file.stat().st_mtime_ns, file.name))
                break
        else:
            raise ValueError(
                "센서 보정 JSON을 찾을 수 없습니다. calibrate_sensors.py를 실행하거나 "
                "--calibration으로 파일을 지정하세요."
            )

    path = path.resolve()
    with path.open(encoding="utf-8-sig") as file:
        try:
            data = json.load(file)
        except json.JSONDecodeError as error:
            raise ValueError(f"센서 보정 JSON 형식 오류. {path} ({error})") from error
    if not isinstance(data, dict) or data.get("spectral_channel", "F2") != "F2":
        raise ValueError(f"F2 센서 보정 JSON을 지정하세요. {path}")

    ids = data.get("sensor_ids")
    if (
        not isinstance(ids, list)
        or any(type(sensor) is not int for sensor in ids)
        or sorted(ids) != list(SENSOR_IDS)
    ):
        raise ValueError(f"센서 보정 JSON에는 ID {list(SENSOR_IDS)}가 한 번씩 있어야 합니다. {path}")

    def read_values(key):
        values = data.get(key)
        if (
            not isinstance(values, list)
            or len(values) != len(ids)
            or any(type(value) not in (int, float) for value in values)
        ):
            raise ValueError(f"{key}에 센서별 숫자가 있어야 합니다. {path}")
        try:
            by_id = {sensor: float(value) for sensor, value in zip(ids, values, strict=True)}
        except OverflowError as error:
            raise ValueError(f"{key}의 수치 범위를 확인하세요. {path}") from error
        return tuple(by_id[sensor] for sensor in SENSOR_IDS)

    offsets = read_values("f2_dark_offsets")
    factors = read_values("f2_correction_factors")
    if any(not 0 <= offset <= 65535 for offset in offsets):
        raise ValueError(f"f2_dark_offsets는 0~65535 사이여야 합니다. {path}")
    if any(not math.isfinite(factor) or factor <= 0 for factor in factors):
        raise ValueError(f"f2_correction_factors는 양의 유한한 값이어야 합니다. {path}")
    return F2Calibration(path, offsets, factors)


def validate_pwm_groups(groups):
    channels = [channel for group in groups for channel in group]
    if (
        len(groups) != GROUP_COUNT
        or any(len(group) != 4 for group in groups)
        or any(type(channel) is not int for channel in channels)
        or sorted(channels) != list(range(1, 25))
    ):
        raise ValueError("PWM_GROUPS에는 채널 1~24가 4개씩 6묶음에 한 번씩 있어야 합니다.")


def validate_settings():
    validate_pwm_groups(PWM_GROUPS)
    if not 0 <= UNIFORMITY_TOLERANCE_PCT < 100:
        raise ValueError("UNIFORMITY_TOLERANCE_PCT는 0 이상 100 미만이어야 합니다.")
    if not 0.1 <= CALIBRATION_PWM_PCT <= MAX_PWM_PCT <= 100:
        raise ValueError("PWM 설정은 0.1 <= CALIBRATION_PWM_PCT <= MAX_PWM_PCT <= 100이어야 합니다.")
    if any(round(value, 1) != value for value in (CALIBRATION_PWM_PCT, MAX_PWM_PCT)):
        raise ValueError("PWM 상한과 초기 측정 PWM은 소수점 한 자리까지 설정하세요.")
    if (
        not math.isfinite(SETTLE_SECONDS)
        or SETTLE_SECONDS < 0
        or not 1 <= SEARCH_REPEATS <= VERIFY_REPEATS
        or any(type(count) is not int for count in (SEARCH_REPEATS, VERIFY_REPEATS))
    ):
        raise ValueError("안정화 시간과 측정 반복 횟수를 확인하세요.")


def expand_pwm(group_pwm, *, groups=None):
    if groups is None:
        groups = PWM_GROUPS
    channels = [0.0] * 24
    for group, duty in zip(groups, group_pwm, strict=True):
        for channel in group:
            channels[channel - 1] = float(duty)
    return channels


class Teensy:
    def __init__(self, port, *, calibration=None):
        self.port = port
        self.calibration = calibration

    def send(self, command):
        data = (command + "\n").encode("ascii")
        if self.port.write(data) != len(data):
            raise RuntimeError(f"시리얼 명령을 끝까지 보내지 못했습니다. {command}")

    def lines(self):
        deadline = time.monotonic() + SERIAL_TIMEOUT_SECONDS
        pending = bytearray()
        while time.monotonic() < deadline:
            pending.extend(self.port.read_until(b"\n", size=512))
            if len(pending) > 512:
                raise RuntimeError("Teensy 시리얼 응답이 너무 깁니다.")
            if pending.endswith(b"\n"):
                yield pending.decode("ascii", errors="replace").strip()
                pending.clear()

    def command(self, command, expected):
        self.send(command)
        for line in self.lines():
            if line == expected:
                return
            if line.startswith(("ERROR", "UNKNOWN COMMAND")):
                raise RuntimeError(f"Teensy 명령 오류 ({command}). {line}")
        raise RuntimeError(f"Teensy 응답 시간 초과 ({command}). 예상 응답은 {expected}입니다.")

    def set_pwm(self, group_pwm):
        self.set_channel_pwm(expand_pwm(group_pwm), SETTLE_SECONDS)

    def set_channel_pwm(self, channel_pwm, settle_seconds):
        self.command(json.dumps(channel_pwm), "OK PWM 24")
        time.sleep(settle_seconds)

    def read_f2_samples(self, repeats):
        samples = []
        for _ in range(repeats):
            reading = self.read_f2()
            if reading is None:
                return None
            samples.append(reading)
        return samples

    def read_f2(self):
        """Read one complete run and apply calibration after raw data validation."""
        self.send("READ")
        run_id = None
        readings = {}
        errors = []
        for line in self.lines():
            fields = line.split(",")
            kind = fields[0]
            if kind == "CSV_BEGIN" and len(fields) == 3 and run_id is None:
                run_id = fields[1]
                continue
            if kind not in ("CSV", "CSV_ERROR", "CSV_END"):
                continue
            if len(fields) < 2 or run_id is None or fields[1] != run_id:
                raise RuntimeError(f"READ 응답의 측정 번호가 일치하지 않습니다. {line}")
            if kind == "CSV" and len(fields) == 13:
                sensor, f2 = int(fields[3]), int(fields[5])
                if sensor not in SENSOR_IDS or sensor in readings or not 0 <= f2 <= 65535:
                    raise RuntimeError(f"READ 센서 데이터가 올바르지 않습니다. {line}")
                readings[sensor] = f2
            elif kind == "CSV_ERROR" and len(fields) == 4:
                errors.append((int(fields[2]), fields[3]))
            elif kind == "CSV_END" and len(fields) == 3:
                failed_ids = [sensor for sensor, _ in errors]
                if any(code != "SATURATED" for _, code in errors):
                    raise RuntimeError(f"Teensy 센서 오류. {errors}")
                reported_ids = sorted([*readings, *failed_ids])
                if (
                    int(fields[2]) != len(readings)
                    or reported_ids != list(SENSOR_IDS)
                ):
                    raise RuntimeError(
                        f"READ 센서 응답 불일치. 예상 ID {list(SENSOR_IDS)}, "
                        f"수신 ID {reported_ids}, 종료 응답 {line}"
                    )
                if errors:
                    return None
                raw_f2 = tuple(readings[sensor] for sensor in SENSOR_IDS)
                if self.calibration is not None:
                    return self.calibration.correct(raw_f2)
                return raw_f2
            else:
                raise RuntimeError(f"READ 응답 형식이 올바르지 않습니다. {line}")
        raise RuntimeError("READ 응답 시간 초과. CSV_END를 받지 못했습니다.")


@contextmanager
def armed_teensy(port_name, *, calibration=None):
    with Serial(port_name, 115200, timeout=0.2, write_timeout=2) as port:
        teensy = Teensy(port, calibration=calibration)
        try:
            time.sleep(2.2)
            port.reset_input_buffer()
            teensy.command("OFF", "ALL PWM OFF / DISARMED")
            teensy.command("ARM", "PWM ARMED")
            yield teensy
        finally:
            interrupted = sys.exc_info()[0] is not None
            try:
                teensy.command("OFF", "ALL PWM OFF / DISARMED")
            except (RuntimeError, SerialException) as error:
                if not interrupted:
                    raise
                print(f"종료 시 OFF 확인 실패. {error}", file=sys.stderr)


def deviation_pct(reading):
    values = np.asarray(reading, dtype=float)
    mean = values.mean()
    return float(np.max(np.abs(values - mean)) * 100.0 / mean) if mean > 0 else math.inf


@dataclass
class Measurement:
    pwm: tuple
    samples: list

    @property
    def f2(self):
        return np.median(self.samples, axis=0)

    @property
    def mean(self):
        return float(self.f2.mean())

    @property
    def deviation(self):
        return deviation_pct(self.f2)

    @property
    def passes(self):
        return self.passes_uniformity(UNIFORMITY_TOLERANCE_PCT)

    def passes_uniformity(self, tolerance_pct):
        return (
            any(self.pwm)
            and self.deviation <= tolerance_pct + 1e-9
            and all(
                deviation_pct(sample) <= tolerance_pct + 1e-9
                for sample in self.samples
            )
        )


def measure(teensy, pwm, repeats):
    teensy.set_pwm(pwm)
    samples = teensy.read_f2_samples(repeats)
    if samples is None:
        print(f"PWM {pwm}  센서 포화", flush=True)
        return None
    result = Measurement(pwm, samples)
    print(
        f"PWM {pwm}  F2 {result.f2.tolist()}  평균 {result.mean:.1f}  "
        f"최대 편차 {result.deviation:.2f}%{'  통과' if result.passes else ''}",
        flush=True,
    )
    return result


def quantize(pwm):
    return tuple(round(float(value), 1) for value in pwm)


class Search:
    def __init__(self, teensy):
        self.teensy = teensy
        self.results = {}

    def measure(self, pwm):
        pwm = quantize(pwm)
        if pwm not in self.results:
            self.results[pwm] = measure(self.teensy, pwm, SEARCH_REPEATS)
        return self.results[pwm]

    def test_candidate(self, pwm):
        if self.measure(pwm) is not None:
            return
        # Find a brighter usable fraction after an explicit saturation response.
        low, high = 0.0, 1.0
        for _ in range(7):
            middle = (low + high) / 2
            scaled = quantize(np.asarray(pwm) * middle)
            if not any(scaled):
                break
            if self.measure(scaled) is None:
                high = middle
            else:
                low = middle

    def measured_candidates(self):
        return [
            result for result in self.results.values()
            if result is not None and any(result.pwm) and result.mean > 0
        ]


def fit_local_model(search, anchor, step):
    """Measure one usable perturbation per group around a measured PWM point."""
    sensitivity = np.empty((len(SENSOR_IDS), GROUP_COUNT))
    origin = anchor.f2
    for group, current in enumerate(anchor.pwm):
        probe_ticks = round(step / PWM_STEP)
        while probe_ticks > 0:
            probe = probe_ticks * PWM_STEP
            for target in (min(MAX_PWM_PCT, current + probe), max(0.0, current - probe)):
                changed = list(anchor.pwm)
                changed[group] = round(target, 1)
                delta = changed[group] - current
                if abs(delta) < PWM_STEP / 2:
                    continue
                sample = search.measure(changed)
                if sample is not None:
                    sensitivity[:, group] = (sample.f2 - origin) / delta
                    break
            else:
                probe_ticks //= 2
                continue
            break
        else:
            print(f"그룹 {group + 1}의 반응을 측정할 수 없습니다. 주변 PWM도 포화됩니다.", flush=True)
            return None
    return origin - sensitivity @ anchor.pwm, sensitivity


def solve_pwm(offset, sensitivity, bounds, tolerance_pct):
    """Maximize the modeled mean F2 under both bounds on every sensor."""
    tolerance = tolerance_pct / 100
    mean_offset = offset.mean()
    mean_sensitivity = sensitivity.mean(axis=0)
    solution = linprog(
        -mean_sensitivity,
        A_ub=np.vstack((
            sensitivity - (1 + tolerance) * mean_sensitivity,
            (1 - tolerance) * mean_sensitivity - sensitivity,
        )),
        b_ub=np.concatenate((
            (1 + tolerance) * mean_offset - offset,
            offset - (1 - tolerance) * mean_offset,
        )),
        bounds=bounds,
        method="highs",
    )
    if solution.status == 2:
        return None
    if not solution.success:
        raise RuntimeError(f"선형계획법 계산 오류. {solution.message}")
    return solution.x


def rounded_candidates(solution, offset, sensitivity, bounds):
    """Rank the floor/ceiling combinations near a continuous LP solution."""
    choices = []
    for value, (low, high) in zip(solution, bounds, strict=True):
        ticks = round(float(value) * 10, 8)
        choices.append(sorted({
            min(high, max(low, math.floor(ticks) / 10)),
            min(high, max(low, math.ceil(ticks) / 10)),
        }))

    def score(pwm):
        predicted = offset + sensitivity @ pwm
        excess = max(0.0, deviation_pct(predicted) - UNIFORMITY_TOLERANCE_PCT - 1e-9)
        return excess, -predicted.mean()

    return sorted(itertools.product(*choices), key=score)[:2]


def search_model(search, offset, sensitivity, bounds):
    # Also test a slightly tighter model target to allow for measurement noise.
    for tolerance in dict.fromkeys((
        UNIFORMITY_TOLERANCE_PCT,
        UNIFORMITY_TOLERANCE_PCT - min(0.5, UNIFORMITY_TOLERANCE_PCT * 0.2),
    )):
        solution = solve_pwm(offset, sensitivity, bounds, tolerance)
        if solution is not None:
            for pwm in rounded_candidates(solution, offset, sensitivity, bounds):
                if any(pwm):
                    search.test_candidate(pwm)


def closest_first(measurement):
    worst_sample = max(deviation_pct(sample) for sample in measurement.samples)
    return max(worst_sample, measurement.deviation), -measurement.mean


def optimize(teensy):
    search = Search(teensy)
    dark = search.measure((0.0,) * GROUP_COUNT)
    if dark is None:
        raise RuntimeError("전체 PWM 0%에서도 센서가 포화되었습니다. 적분시간이나 주변광을 확인하세요.")

    print("그룹별 반응 측정 및 선형계획법 계산", flush=True)
    model = fit_local_model(search, dark, CALIBRATION_PWM_PCT)
    if model is None:
        raise RuntimeError("초기 반응을 측정하지 못했습니다. 적분시간을 줄여 다시 실행하세요.")
    search_model(search, *model, [(0.0, MAX_PWM_PCT)] * GROUP_COUNT)

    # Seed a combined low-PWM setting when the initial model has no usable solution.
    if not any(result.passes for result in search.measured_candidates()):
        search.test_candidate((CALIBRATION_PWM_PCT,) * GROUP_COUNT)

    for radius in (10.0, 3.0, 1.0):
        candidates = search.measured_candidates()
        if not candidates:
            break
        passing = [result for result in candidates if result.passes]
        anchor = max(passing, key=lambda result: result.mean) if passing else min(
            candidates, key=closest_first
        )
        print(f"실측 기준점 재측정 및 주변 탐색 (±{radius:g}%p)", flush=True)
        refreshed = measure(teensy, anchor.pwm, SEARCH_REPEATS)
        search.results[anchor.pwm] = refreshed
        if refreshed is None:
            continue
        model = fit_local_model(search, refreshed, radius / 2)
        if model is None:
            continue
        bounds = [
            (round(max(0.0, value - radius), 1), round(min(MAX_PWM_PCT, value + radius), 1))
            for value in anchor.pwm
        ]
        search_model(search, *model, bounds)

    candidates = search.measured_candidates()
    print(f"최종 검증 ({VERIFY_REPEATS}회 반복)", flush=True)
    verified = None
    for candidate in sorted(
        (result for result in candidates if result.passes),
        key=lambda result: result.mean,
        reverse=True,
    ):
        result = measure(teensy, candidate.pwm, VERIFY_REPEATS)
        search.results[candidate.pwm] = result
        if result is not None and result.passes:
            if verified is None or result.mean > verified.mean:
                verified = result
    closest = min(search.measured_candidates(), key=closest_first, default=None)
    return verified, closest


def result_data(measurement, verified, *, groups, tolerance_pct=None):
    data = {
        "status": "verified" if verified else "no_verified_solution",
    }
    if tolerance_pct is not None:
        data["uniformity_tolerance_pct"] = tolerance_pct
    data.update(pwm_groups=groups, sensor_ids=SENSOR_IDS)
    if measurement is not None:
        data.update(
            group_pwm_pct=measurement.pwm,
            channel_pwm_pct=expand_pwm(measurement.pwm, groups=groups),
            sensor_f2_median=measurement.f2.tolist(),
            mean_f2=measurement.mean,
            max_deviation_pct=measurement.deviation,
            worst_repeat_deviation_pct=max(map(deviation_pct, measurement.samples)),
            f2_repeats=measurement.samples,
        )
    # A zero-light sample has undefined relative uniformity.
    for key in ("max_deviation_pct", "worst_repeat_deviation_pct"):
        if key in data and not math.isfinite(data[key]):
            data[key] = None
    return data


def write_result(path, data, *, calibration=None):
    if calibration is not None:
        data = {**data, "sensor_calibration": calibration.metadata()}
    with path.open("x", encoding="utf-8") as output:
        json.dump(data, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")


def save_result(path, measurement, verified, *, calibration=None):
    data = result_data(
        measurement, verified, groups=PWM_GROUPS, tolerance_pct=UNIFORMITY_TOLERANCE_PCT
    )
    write_result(path, data, calibration=calibration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=SERIAL_PORT, help="Teensy 시리얼 포트")
    parser.add_argument("--output", type=Path, help="새 결과 JSON 파일 경로")
    parser.add_argument("--calibration", type=Path, help="센서 보정 JSON (기본값은 최신 파일)")
    args = parser.parse_args()
    output = args.output or Path(f"optimized_pwm_{datetime.now():%Y%m%d_%H%M%S_%f}.json")
    try:
        validate_settings()
        calibration = load_f2_calibration(args.calibration)
        if output.exists() or not output.parent.is_dir():
            raise ValueError(f"기존 파일과 겹치지 않는 결과 경로를 지정하세요. {output}")
        print(f"F2 보정 파일 {calibration.path}", flush=True)
        with armed_teensy(args.port, calibration=calibration) as teensy:
            verified, closest = optimize(teensy)
        save_result(output, verified or closest, verified is not None, calibration=calibration)
    except KeyboardInterrupt:
        print("사용자가 탐색을 중단했습니다.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"최적화 중단. {error}", file=sys.stderr)
        return 1

    if verified is not None:
        print(f"검증 완료. 최대 편차 {verified.deviation:.2f}% · 평균 F2 {verified.mean:.1f}")
        print(f"CH1~24 PWM (%) {expand_pwm(verified.pwm)}")
    else:
        print(f"±{UNIFORMITY_TOLERANCE_PCT:g}%를 만족하는 조합을 검증하지 못했습니다.")
        if closest is not None:
            print(f"가장 가까운 실측 조합의 최대 편차는 {closest_first(closest)[0]:.2f}%입니다.")
    print(f"결과 파일 {output.resolve()}")
    return 0 if verified is not None else 2


if __name__ == "__main__":
    sys.exit(main())

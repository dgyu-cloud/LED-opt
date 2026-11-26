"""Calibrate four AS7341 sensors against the same light using their F2 readings."""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from statistics import median

from serial import Serial


SERIAL_PORT = "COM3"
SENSOR_IDS = (0, 1, 2, 3)
REFERENCE_SENSOR = 0
CALIBRATION_PWM_PCT = 50.0
MEASUREMENT_REPEATS = 10
LED_SETTLE_SECONDS = 0.3
SERIAL_TIMEOUT_SECONDS = 12.0


def response_lines(port):
    deadline = time.monotonic() + SERIAL_TIMEOUT_SECONDS
    pending = bytearray()
    while time.monotonic() < deadline:
        pending.extend(port.read_until(b"\n", size=256))
        if len(pending) > 512:
            raise RuntimeError("Teensy 시리얼 응답이 너무 깁니다.")
        if pending.endswith(b"\n"):
            yield pending.decode("ascii", errors="replace").strip()
            pending.clear()


def send_command(port, command):
    payload = f"{command}\n".encode("ascii")
    if port.write(payload) != len(payload):
        raise RuntimeError(f"Teensy 명령을 끝까지 보내지 못했습니다. {command}")


def expect_command(port, command, expected):
    send_command(port, command)
    for line in response_lines(port):
        if line == expected:
            return
        if line.startswith(("ERROR", "UNKNOWN COMMAND")):
            raise RuntimeError(f"Teensy 명령 오류 ({command}). {line}")
    raise RuntimeError(f"Teensy 응답 시간 초과 ({command}). 예상 응답은 {expected}입니다.")


def read_f2(port, sensor):
    send_command(port, f"READ {sensor}")
    run_id = None
    f2 = None
    errors = []
    for line in response_lines(port):
        if line.startswith(("ERROR", "UNKNOWN COMMAND")):
            raise RuntimeError(f"센서 {sensor} 측정 명령 오류. {line}")
        fields = line.split(",")
        kind = fields[0]
        if kind == "CSV_BEGIN" and len(fields) == 3 and run_id is None:
            run_id = fields[1]
            continue
        if kind not in ("CSV", "CSV_ERROR", "CSV_END"):
            continue
        if len(fields) < 2 or run_id is None or fields[1] != run_id:
            raise RuntimeError(f"센서 {sensor}의 측정 번호가 일치하지 않습니다. {line}")
        if kind == "CSV" and len(fields) == 13:
            value = int(fields[5])
            if int(fields[3]) != sensor or f2 is not None or not 0 <= value <= 65535:
                raise RuntimeError(f"센서 {sensor}의 응답값이 올바르지 않습니다. {line}")
            f2 = value
        elif kind == "CSV_ERROR" and len(fields) == 4 and int(fields[2]) == sensor:
            errors.append(fields[3])
        elif kind == "CSV_END" and len(fields) == 3:
            if errors:
                raise RuntimeError(f"센서 {sensor} 측정 실패. {', '.join(errors)}")
            if int(fields[2]) == 1 and f2 is not None:
                return f2
            raise RuntimeError(f"센서 {sensor}의 F2 값이 누락되었습니다. {line}")
        else:
            raise RuntimeError(f"센서 {sensor}의 응답 형식이 올바르지 않습니다. {line}")
    raise RuntimeError(f"센서 {sensor}의 측정 시간이 초과되었습니다.")


def measure_f2(port, sensor):
    samples = [read_f2(port, sensor) for _ in range(MEASUREMENT_REPEATS)]
    return median(samples)


def leds_off(port):
    expect_command(port, "OFF", "ALL PWM OFF / DISARMED")


def measure_lit_f2(port, sensor):
    try:
        expect_command(port, "ARM", "PWM ARMED")
        duty = f"{CALIBRATION_PWM_PCT:.1f}"
        expect_command(port, f"ALL {duty}", f"ALL PWM = {duty}%")
        time.sleep(LED_SETTLE_SECONDS)
        return measure_f2(port, sensor)
    finally:
        original_error = sys.exception()
        try:
            leds_off(port)
        except (OSError, RuntimeError) as error:
            message = f"보정 후 LED OFF 확인 실패. {error}"
            if original_error is None:
                raise RuntimeError(message) from error
            print(message, file=sys.stderr)


def calibrate(port):
    dark_offsets = []
    light_medians = []

    leds_off(port)
    print("센서 0~3을 같은 위치와 각도에 하나씩 놓습니다.")
    print(f"암값 측정 후 전체 24채널을 {CALIBRATION_PWM_PCT:g}%로 켜서 F2를 측정합니다.")
    for sensor in SENSOR_IDS:
        input(f"\n센서 {sensor}를 기준 위치에 놓고 감광부를 차광한 뒤 Enter를 누르세요. ")
        dark = measure_f2(port, sensor)
        dark_offsets.append(dark)
        print(f"센서 {sensor} 암값 F2 중앙값 {dark:g}", flush=True)

        input("센서 위치를 유지하고 차광을 제거한 뒤 Enter를 누르세요. ")
        f2 = measure_lit_f2(port, sensor)
        print(f"센서 {sensor} 점등 F2 중앙값 {f2:g}", flush=True)
        if f2 <= dark:
            raise RuntimeError(f"센서 {sensor}의 점등 F2가 암값 이하입니다. 센서 위치와 차광을 확인하세요.")
        light_medians.append(f2)

    net = [lit - dark for dark, lit in zip(dark_offsets, light_medians, strict=True)]
    reference = net[SENSOR_IDS.index(REFERENCE_SENSOR)]
    return {
        "sensor_ids": SENSOR_IDS,
        "reference_sensor_id": REFERENCE_SENSOR,
        "spectral_channel": "F2",
        "led_pwm_pct": CALIBRATION_PWM_PCT,
        "samples_per_point": MEASUREMENT_REPEATS,
        "f2_dark_offsets": dark_offsets,
        "f2_correction_factors": [reference / value for value in net],
        "f2_light_medians": light_medians,
        "correction_formula": "(raw_f2 - f2_dark_offset) * f2_correction_factor",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=SERIAL_PORT, help="Teensy 시리얼 포트")
    parser.add_argument("--output", type=Path, help="새 보정 결과 JSON 경로")
    args = parser.parse_args()
    output = args.output or Path(f"sensor_calibration_{datetime.now():%Y%m%d_%H%M%S_%f}.json")

    try:
        if REFERENCE_SENSOR not in SENSOR_IDS:
            raise ValueError("REFERENCE_SENSOR는 센서 ID 0~3 중 하나여야 합니다.")
        if not 0.1 <= CALIBRATION_PWM_PCT <= 100 or round(CALIBRATION_PWM_PCT, 1) != CALIBRATION_PWM_PCT:
            raise ValueError("CALIBRATION_PWM_PCT는 0.1~100 사이이며 소수점 한 자리까지 설정할 수 있습니다.")
        if type(MEASUREMENT_REPEATS) is not int or MEASUREMENT_REPEATS < 1:
            raise ValueError("MEASUREMENT_REPEATS는 양의 정수여야 합니다.")
        if output.exists() or not output.parent.is_dir():
            raise ValueError(f"기존 파일과 겹치지 않는 결과 경로를 지정하세요. {output}")
        with Serial(args.port, 115200, timeout=0.2, write_timeout=2) as port:
            port.reset_input_buffer()
            result = calibrate(port)
        with output.open("x", encoding="utf-8") as file:
            json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
            file.write("\n")
    except (KeyboardInterrupt, EOFError):
        print("센서 보정을 중단했습니다.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"센서 보정 실패. {error}", file=sys.stderr)
        return 1

    for sensor, dark, factor in zip(
        SENSOR_IDS, result["f2_dark_offsets"], result["f2_correction_factors"], strict=True
    ):
        print(f"센서 {sensor}  암값 {dark:g}  보정계수 {factor:.6f}")
    print(f"보정 결과 {output.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

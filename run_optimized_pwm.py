
channel_pwm =[
    100.0,
    100.0,
    92.6,
    92.6,
    100.0,
    100.0,
    61.2,
    80.8,
    40.2,
    40.2,
    80.8,
    61.2,
    61.2,
    80.8,
    40.2,
    40.2,
    80.8,
    61.2,
    100.0,
    100.0,
    92.6,
    92.6,
    100.0,
    100.0
  ]
channel_pwm =[
    100 for i in range(24)
  ]
main_power = 0.5 # 1.0 = configured PWM, 0.5 = half PWM.
serial_port = "COM3"

import argparse
import json
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

from serial import Serial


SENSOR_INTERVAL_SECONDS = 1.0
SENSOR_IDS = (0, 1, 2, 3)
SENSOR_CHANNELS = ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "Clear")
SERIAL_TIMEOUT_SECONDS = 12.0


def load_f2_calibration(path):
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

    offsets = data.get("f2_dark_offsets")
    factors = data.get("f2_correction_factors")
    for values in (offsets, factors):
        if (
            not isinstance(values, list)
            or len(values) != len(ids)
            or any(type(value) not in (int, float) for value in values)
        ):
            raise ValueError(f"보정 JSON에 센서별 암값과 보정계수가 있어야 합니다. {path}")
    if any(not 0 <= offset <= 65535 for offset in offsets):
        raise ValueError(f"f2_dark_offsets는 0~65535 사이여야 합니다. {path}")
    if any(not math.isfinite(factor) or factor <= 0 for factor in factors):
        raise ValueError(f"f2_correction_factors는 양의 유한한 값이어야 합니다. {path}")
    return path, {
        sensor: (offset, factor)
        for sensor, offset, factor in zip(ids, offsets, factors, strict=True)
    }


def scale_pwm(channels, power):
    if len(channels) != 24:
        raise ValueError("channel_pwm에 CH1~CH24 순서로 값 24개를 넣으세요.")
    if any(type(duty) not in (int, float) or not 0 <= duty <= 100 for duty in channels):
        raise ValueError("채널별 PWM은 0~100 사이의 숫자여야 합니다.")
    if type(power) not in (int, float) or not math.isfinite(power) or power < 0:
        raise ValueError("main_power는 0 이상의 유한한 숫자여야 합니다.")
    scaled = [power * duty for duty in channels]
    if any(duty > 100 for duty in scaled):
        raise ValueError("main_power를 곱한 PWM이 100%를 넘습니다.")
    return scaled


def send_command(port, text):
    payload = (text + "\n").encode("ascii")
    if port.write(payload) != len(payload):
        raise RuntimeError("시리얼 명령을 끝까지 보내지 못했습니다.")


def response_lines(port):
    deadline = time.monotonic() + SERIAL_TIMEOUT_SECONDS
    line = bytearray()
    while time.monotonic() < deadline:
        line.extend(port.read_until(b"\n", size=256))
        if len(line) > 512:
            raise RuntimeError("Teensy 응답이 너무 깁니다.")
        if not line.endswith(b"\n"):
            continue
        response = line.decode("ascii", errors="replace").strip()
        line.clear()
        if response.startswith(("ERROR", "UNKNOWN COMMAND")):
            raise RuntimeError(f"Teensy 명령 오류. {response}")
        yield response


def command(port, text, expected):
    send_command(port, text)
    for response in response_lines(port):
        if response == expected:
            return
    raise RuntimeError(f"Teensy 응답 시간 초과. 예상 응답은 {expected}입니다.")


def read_sensors(port):
    send_command(port, "READ")
    run_id = None
    readings = {}
    errors = {}
    for response in response_lines(port):
        fields = response.split(",")
        kind = fields[0]
        if kind == "CSV_BEGIN" and len(fields) == 3:
            run_id = fields[1]
            readings.clear()
            errors.clear()
            continue
        if run_id is None or len(fields) < 2 or fields[1] != run_id:
            continue
        if kind == "CSV" and len(fields) == 4 + len(SENSOR_CHANNELS):
            readings[int(fields[3])] = tuple(map(int, fields[4:]))
        elif kind == "CSV_ERROR" and len(fields) == 4:
            errors.setdefault(int(fields[2]), []).append(fields[3])
        elif kind == "CSV_END" and len(fields) == 3:
            if int(fields[2]) != len(readings):
                raise RuntimeError(f"Teensy 센서 응답 개수가 일치하지 않습니다. {response}")
            return readings, errors
    raise RuntimeError("Teensy 센서 응답 시간 초과. CSV_END를 받지 못했습니다.")


def print_sensors(readings, errors, calibration):
    timestamp = datetime.now().isoformat(sep=" ", timespec="seconds")
    lines = [f"[{timestamp}]"]
    for sensor in SENSOR_IDS:
        if sensor in readings:
            values = zip(SENSOR_CHANNELS, readings[sensor], strict=True)
            dark, factor = calibration[sensor]
            fields = []
            for channel, value in values:
                if channel == "F2":
                    corrected = (value - dark) * factor
                    if not math.isfinite(corrected):
                        raise ValueError(f"센서 {sensor}의 보정 F2가 유한한 값이어야 합니다.")
                    fields.append(f"F2={corrected:.2f} (F2_raw={value})")
                else:
                    fields.append(f"{channel}={value}")
            details = " ".join(fields)
        else:
            details = "측정값 없음"
        if sensor in errors:
            details += f" ({', '.join(errors[sensor])})"
        lines.append(f"  센서 {sensor}  {details}")
    print("\n".join(lines), flush=True)


def run_pwm(port_name, pwm, calibration):
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    stop_signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, "SIGBREAK"):
        stop_signals.append(signal.SIGBREAK)
    previous_handlers = {
        sig: signal.signal(sig, request_stop) for sig in stop_signals
    }

    try:
        with Serial(port_name, 115200, timeout=0.2, write_timeout=2) as port:
            try:
                port.reset_input_buffer()
                command(port, "OFF", "ALL PWM OFF / DISARMED")
                if stopping:
                    return
                command(port, "ARM", "PWM ARMED")
                if stopping:
                    return
                # Keep the line compact. The firmware applies its 0.1% PWM resolution.
                pwm_line = "[" + ",".join(f"{duty:.9g}" for duty in pwm) + "]"
                command(port, pwm_line, "OK PWM 24")
                print(f"LED PWM (%) {pwm_line}", flush=True)
                print("종료하려면 Ctrl+C를 누르세요.", flush=True)
                next_read = time.monotonic()
                while not stopping:
                    now = time.monotonic()
                    if now < next_read:
                        time.sleep(min(0.2, next_read - now))
                        continue
                    next_read = now + SENSOR_INTERVAL_SECONDS
                    print_sensors(*read_sensors(port), calibration)
            finally:
                original_error = sys.exception()
                try:
                    command(port, "OFF", "ALL PWM OFF / DISARMED")
                except (OSError, RuntimeError) as error:
                    if original_error is None:
                        raise RuntimeError(f"종료 시 LED OFF 확인 실패. {error}") from error
                    print(f"종료 시 LED OFF 확인 실패. {error}", file=sys.stderr)
                else:
                    print("전체 LED OFF 확인.", flush=True)
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description="Teensy에 설정한 24채널 PWM을 적용합니다.")
    parser.add_argument("--port", default=serial_port, help="Teensy 시리얼 포트")
    parser.add_argument("--calibration", type=Path, help="센서 보정 JSON (기본값은 최신 파일)")
    args = parser.parse_args()
    try:
        path, calibration = load_f2_calibration(args.calibration)
        pwm = scale_pwm(channel_pwm, main_power)
        print(f"F2 보정 파일 {path}", flush=True)
        run_pwm(args.port, pwm, calibration)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, OverflowError) as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

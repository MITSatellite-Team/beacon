import json
import sys
import time
import requests
import serial
from datetime import datetime, timezone
from pathlib import Path

PREFIX = "[DECODED DATA]"

DEFAULT_PORT = "/dev/ttyUSB0"   # Change to e.g. "COM3" on Windows
DEFAULT_BAUD = 115200
DEFAULT_URL  = "http://35.222.71.40:8080/api/update"
DEFAULT_LOG  = "serial_log.jsonl"   # <-- output file

RECONNECT_DELAY = 5


def save_to_file(log_path: Path, payload: str) -> None:
    """Append a timestamped record to the JSONL log file."""
    try:
        body = json.loads(payload.strip())
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "data": body,
        }
        with log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")
        print(f"  → Saved to {log_path}")
    except json.JSONDecodeError as exc:
        print(f"  ✗ Could not parse payload for saving: {exc}")
    except OSError as exc:
        print(f"  ✗ File write error: {exc}")


def post_data(url: str, payload: str, session: requests.Session) -> None:
    """Send payload string as JSON body { "data": "<payload>" }."""
    body = json.loads(payload.strip())

    try:
        response = session.post(url, json=body, timeout=10)
        response.raise_for_status()
        print(f"  → POST {url}  [{response.status_code}]")
    except requests.exceptions.ConnectionError:
        print(f"  ✗ Connection error — is the server running at {url}?")
    except requests.exceptions.Timeout:
        print(f"  ✗ Request timed out ({url})")
    except requests.exceptions.HTTPError as exc:
        print(f"  ✗ Server returned error: {exc}")


def open_serial(port: str, baud: int) -> serial.Serial:
    """Open the serial port, blocking until successful."""
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"[serial] Opened {port} @ {baud} baud")
            return ser
        except serial.SerialException as exc:
            print(f"[serial] Cannot open {port}: {exc}  — retrying in {RECONNECT_DELAY}s")
            time.sleep(RECONNECT_DELAY)


def run(port: str, baud: int, url: str, log_path: Path) -> None:
    session = requests.Session()

    while True:                          # outer loop: reconnect on serial loss
        ser = open_serial(port, baud)
        try:
            print(f"[main] Listening for lines starting with '{PREFIX}' …\n")
            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as exc:
                    print(f"\n[serial] Read error: {exc} — reconnecting …")
                    break                # break inner → reconnect via outer loop

                if not raw:
                    continue             # timeout / empty read

                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                except Exception:
                    continue

                if line.startswith(PREFIX):
                    payload = line[len(PREFIX):]   # everything after the prefix
                    save_to_file(log_path, payload)
                    post_data(url, payload, session)

        finally:
            ser.close()
            print("[serial] Port closed")


log_path = Path(DEFAULT_LOG)
print(f"[main] Logging decoded data to: {log_path.resolve()}")

while True:
    try:
        run(DEFAULT_PORT, DEFAULT_BAUD, DEFAULT_URL, log_path)
    except KeyboardInterrupt:
        print("\n[main] Stopped by user.")
        sys.exit(0)
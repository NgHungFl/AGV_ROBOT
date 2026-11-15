
import serial
import time
import re

# ==============================
# CAU HINH
# ==============================

COM_PORT = "COM6"       # SUA THANH COM CUA BAN
BAUDRATE = 115200

# ==============================
# KET NOI UART
# ==============================

try:
    ser = serial.Serial(
        port=COM_PORT,
        baudrate=BAUDRATE,
        timeout=1
    )

    time.sleep(2)
    ser.reset_input_buffer()

    print("================================")
    print("     LINE SENSOR MONITOR")
    print("================================")
    print(f"Connected: {COM_PORT}")
    print(f"Baudrate : {BAUDRATE}")
    print("Dang doc du lieu...\n")

except serial.SerialException as e:
    print("KHONG THE KET NOI UART!")
    print("Kiem tra lai COM_PORT.")
    print("Chi tiet:", e)
    input("\nNhan Enter de thoat...")
    raise SystemExit


# ==============================
# DOC DU LIEU
# ==============================

raw_values = [0, 0, 0, 0, 0]
normalized_values = [0, 0, 0, 0, 0]
position = "---"
state = "WAITING"


def read_values(line, label):
    pattern = rf"{label}\s*:\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)"

    match = re.search(pattern, line)

    if match:
        return [int(x) for x in match.groups()]

    return None


try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(
                "utf-8", errors="ignore"
            ).strip()

            if not line:
                continue

            values = read_values(line, "RAW")
            if values is not None:
                raw_values = values

            values = read_values(line, "VAL")
            if values is not None:
                normalized_values = values

            pos_match = re.search(r"POS\s*:\s*(-?\d+|---)", line)
            if pos_match:
                position = pos_match.group(1)

            if "LINE DETECTED" in line:
                state = "LINE DETECTED"

            elif "LINE LOST" in line:
                state = "LINE LOST"

            # In khi nhan duoc dong STATE
            if line.startswith("STATE"):
                print("\n--------------------------------")
                print("SENSOR       S1    S2    S3    S4    S5")
                print("RAW       :", " ".join(f"{x:5d}" for x in raw_values))
                print("NORMALIZED:", " ".join(f"{x:5d}" for x in normalized_values))
                print("--------------------------------")
                print(f"LINE POSITION: {position}")
                print(f"LINE STATUS  : {state}")
                print("--------------------------------")

except KeyboardInterrupt:
    print("\nDa dung chuong trinh.")

finally:
    ser.close()

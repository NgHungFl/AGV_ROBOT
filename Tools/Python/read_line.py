
import serial
import time

# ==============================
# CAU HINH UART
# ==============================

PORT = "COM6"
BAUDRATE = 115200
TIMEOUT = 1

# ==============================
# KET NOI STM32
# ==============================

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT
    )

    time.sleep(2)

    print("===== AGV SENSOR MONITOR =====")
    print(f"COM: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print("-----------------------------------")
    print("LINE SENSOR")
    print("AIN1   AIN2   AIN3   AIN4   AIN5")
    print("-----------------------------------")

    while True:

        data = ser.readline().decode(
            "utf-8", errors="ignore"
        ).strip()

        if not data:
            continue

        # ==============================
        # DOC CAM BIEN MAU TCS34725
        # ==============================

        if data.startswith("COLOR,"):

            parts = data.split(",")

            if len(parts) == 6:
                try:
                    red = int(parts[1])
                    green = int(parts[2])
                    blue = int(parts[3])
                    clear = int(parts[4])
                    color = parts[5]

                    print("\n===== COLOR SENSOR =====")
                    print(f"Red   : {red}")
                    print(f"Green : {green}")
                    print(f"Blue  : {blue}")
                    print(f"Clear : {clear}")
                    print(f"Color : {color}")
                    print("========================")

                except ValueError:
                    continue

        # ==============================
        # DOC 5 CAM BIEN LINE
        # ==============================

        # else:
        #     try:
        #         values = [int(x) for x in data.split(",")]

        #         if len(values) == 5:
        #             print(
        #                 f"{values[4]:5d}  "
        #                 f"{values[3]:5d}  "
        #                 f"{values[2]:5d}  "
        #                 f"{values[1]:5d}  "
        #                 f"{values[0]:5d}"
        #             )

        #     except ValueError:
        #         continue

except serial.SerialException as e:
    print(f"Loi ket noi {PORT}: {e}")

except KeyboardInterrupt:
    print("\nDa dung chuong trinh.")

finally:
    if "ser" in locals() and ser.is_open:
        ser.close()


import serial
import time

PORT = "COM6"
BAUDRATE = 115200
TIMEOUT = 1

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT
    )

    time.sleep(2)

    print("===== UART MONITOR =====")
    print("Port:", PORT)
    print("Baudrate:", BAUDRATE)
    print("Waiting for STM32 data...\n")

    while True:
        data = ser.readline().decode(
            "utf-8",
            errors="replace"
        ).strip()

        if data:
            print("STM32 >>", repr(data))

except serial.SerialException as e:
    print("UART CONNECTION ERROR:", e)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("UART closed.")

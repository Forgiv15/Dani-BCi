import serial
import time

PORT = "COM5"

with serial.Serial(
    PORT,
    115200,
    timeout=1,
    rtscts=False,
    dsrdtr=False,
) as ser:

    # Deassert both active-low control signals
    ser.dtr = False
    ser.rts = False

    print("DTR=False, RTS=False")
    print("Measure both CP2102N pins now.")
    time.sleep(10)

    # Assert both active-low control signals
    ser.dtr = True
    ser.rts = True

    print("DTR=True, RTS=True")
    print("Measure both CP2102N pins again.")
    time.sleep(10)
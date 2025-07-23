import serial
import time

def wait_for_complete(ser:serial.Serial,count):
    buf = []
    while True:
        buf += ser.readall()
        time.sleep(0.1)
        if len(buf) >= count:
            break

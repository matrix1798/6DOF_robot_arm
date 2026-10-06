import time
import serial
import serial.tools.list_ports
from code.tools.kinematicTools import robot6DOF

ports = serial.tools.list_ports.comports()

if not ports:
    print("Not found any connected ports.")
else:
    for port in ports:
        print(f"Port: {port.device}")
        print(f"Description: {port.description}")
        print(f"HWID: {port.hwid}")
        print('-' * 30)

for port in ports:
    if "Silicon Labs" in port.description: PORT = port.device

BAUD_RATE = 115200

servo_driver = serial.Serial(PORT, BAUD_RATE, timeout=1)
servo_driver.setDTR(False)
servo_driver.setRTS(False)

#without this when PC open a conn port he send a reset signal to esp32
print("Waiting for statr up the esp32...")
time.sleep(8)

"""
positions = [1010,1000]
msg = f"<{','.join(map(str,positions))}>"

servo_driver.write(msg.encode("utf-8"))
print(f"Send message: {msg}")
"""

try:
    while True:
        if servo_driver.in_waiting > 0:

            msg = servo_driver.readline().decode('utf-8', errors='ignore').strip()
            print(msg)

except KeyboardInterrupt:
    print('Closing port...')
    servo_driver.close()
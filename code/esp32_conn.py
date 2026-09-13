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

servo_driver = serial.Serial(PORT, BAUD_RATE)
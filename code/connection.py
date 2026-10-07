import time
import serial
import serial.tools.list_ports

class servoConect:

    def __init__(self):

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

        self.servo_driver = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
        self.servo_driver.setDTR(False)
        self.servo_driver.setRTS(False)

        print("Waiting for start up the esp32...")
        time.sleep(8)

        self.id_array = [0,0,0,0,0,0]
        self.pos_array = [0,0,0,0,0,0]
        self.temp_array = [0,0,0,0,0,0]
        self.voltage_array = [0,0,0,0,0,0]
        self.current_array = [0,0,0,0,0,0]
        
    def recvFeedback(self):
        if self.servo_driver is None:
            return
        
        while True:
            try:
                if self.servo_driver.in_waiting > 0:
                    
                    msg = self.servo_driver.readline().decode('utf-8', errors='ignore').strip()
                    print(msg)

                    if msg and msg[0] == '<' and msg[-1] == '>':
                        msg = msg[1:-1]
                        servos_msg = msg.split(';')
                        servos_msg = servos_msg[:-1]

                        for id, servo_msg in enumerate(servos_msg):
                            if id > 5:
                                break

                            feedback = servo_msg.split(',')
                            #print(feedback)
                            
                            self.id_array[id] = feedback[0]
                            self.pos_array[id] = feedback[1]
                            self.temp_array[id] = feedback[2]
                            self.voltage_array[id] = feedback[3]
                            self.current_array[id] = feedback[4]
                else:
                    time.sleep(0.05)

            except Exception as e:
                print(f"Serial port read error: {e}")
                time.sleep(1)
                
    def sendMessage(self, positions, speed):
        msg = f"<{speed}|{','.join(map(str, positions))}>"
        self.servo_driver.write(msg.encode("utf-8"))   



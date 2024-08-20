import time
import serial

com_port = 'COM3' # /dev/ttyUSB0 #  choose if linux or windows
bd_rate = 115200
def send_command(command):
    print("Pi: command sent: " + command)
    ser.write((command + '\n').encode('utf-8'))
    read_response()


def read_response():
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print("Received from M5Stick: ", response)
            break


def set_max_current(maximal_current = 800): # in mA
    print("Pi: command sent: Set max current to " + str(maximal_current))
    ser.write(("Max current " + str(maximal_current) + '\n').encode('utf-8'))
    read_response()

if __name__ == "__main__":
    commands = ["Init", "Measure current 0", "Measure current 1", "Measure current 2", "Switch 1 on", "Switch 1 off", "Switch 2 on", "Switch 2 off", "Get switch 1 state", "Get switch 2 state"]
    commands = ["Measure current 0", "Measure current 1", "Measure current 2", "Switch 1 on", "Switch 1 off", "Switch 2 on", "Switch 2 off"]
    commands = ["Switch 1 on"]

    ser = serial.Serial(com_port, bd_rate, timeout = 1)  # Change to the correct COM port for your system

    time.sleep(2)  # Wait for the connection to be established
    ser.flushInput()
    ser.flushOutput()

    for command in commands:
        send_command(command)
        # read_response()
        time.sleep(5)
    set_max_current(700)
    set_max_current(1200)
    time.sleep(10)



    ser.close()

# @file     epuck-communication.py
# @brief    E-puck 2 communication to computer (computer side).
# @note     We work in little endian because chSequentialStreamWrite from 
#           ChibiOS sends data in little endian when working with multibyte data

import sys
import serial
import time
import numpy as np
import struct
from threading import Thread
from PIL import Image # read image

# ===========================================================================
#  Module constants.                                                         
# ===========================================================================

# Serial 
SERIAL_PORT_DEFAULT         = "COM8"  
SERIAL_BAUD_RATE            = 115200

# Error, timeout
TIMEOUT_PERIOD              = 1
ERROR_COUNT_MAX             = 10

# Periods
READ_PERIOD                 = 0.1
WRITE_PERIOD                = 0.1

# Commands
COMMANDS = (
    'R'     ,   # RESET
    'P'     ,   # PAUSE
    'C'     ,   # CONTINUE
    'B'     ,   # BEGIN CALIBRATION
    'G'     ,   # GET DATA
    'D'     ,   # DRAW
    'I'         # INTERACTIVE
)


# ===========================================================================
#  Threads.                                                         
# ===========================================================================

# class serial_thread(Thread):

#     def __init__(self, port): # constructor
#         Thread.__init__(self)
#         # class variables

#         # attempt conneciton to e-puck
#         error_count = 0
#         while error_count < ERROR_COUNT_MAX:
#             print("Attempting connection to " + port + "...")
#             try:
#                 self.ser = serial.Serial(port, SERIAL_BAUD_RATE, 
#                 timeout = TIMEOUT_PERIOD)
#                 print("Connection established on " + port)
#                 break
#             except serial.SerialException:
#                 print("Cannot establish connection to the e-puck."
#                 " Retrying " + str(error_count) + "/" + str(ERROR_COUNT_MAX))
#                 error_count += 1
#                 time.sleep(TIMEOUT_PERIOD)
#         if error_count == ERROR_COUNT_MAX:
#             print("Connection failed on " + port +
#             ". Verify that the e-puck is connected to Bluetooth.")
#             sys.exit(0)
#         # discard input and ouput buffers
#         self.ser.flushInput()
#         self.ser.flushOutput()

#     # def run(self):

#     #     while(self.alive):

def receive_image(ser):
    state = 0

    while(state != 5):
        #reads 1 byte
        c1 = ser.read(1)
        #timeout condition
        if(c1 == b''):
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    print("Receiving image")

    msg = ser.readline().decode("utf_8")
    print(msg)
    length = struct.unpack('<h',ser.read(2))  # length is sent as an uint16
    length = length[0]
    print("length" + str(length))

    x_buffer = b''
    y_buffer = b''
    c_buffer = b''

    x_buffer = ser.read(length)
    y_buffer = ser.read(length)
    c_buffer = ser.read(length)

    c_buffer += b'\x00'
    i = 0
    for i in range(0, length):
        # print("%d x %d " %(i, x_buffer[i]))
        print("%d x %d y %d C %d" % (i, x_buffer[i], y_buffer[i], c_buffer[i]))

    f = open("out.svg",'w')
    f.write(makesvg(x_buffer, y_buffer, c_buffer, length))
    f.close()
    
#     toRead = 4000
#     rcv_buffer = b''
#     # print(rcv_buffer)
#     # print(len(rcv_buffer))
#     #reads the data
#     i = 0
#     # while i < toRead:
#     #     print(ser.read())
#     #     i +=1

# # WORKING 90X100 rgb
#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(2000)

#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(1000)

#     new_buffer = bytearray(len(rcv_buffer))
#     new_buffer[0::2] = rcv_buffer[1::2]
#     new_buffer[1::2] = rcv_buffer[0::2]
#     im = Image.frombytes("RGB", (100, 90), bytes(new_buffer), "raw", "BGR;16")
#     # im = Image.frombytes("L", (100, 90), bytes(rcv_buffer), "raw")
#     nameimg ="Image"
#     im.show(title=nameimg)
#     im.save("C:/Users/41786/Desktop/Projects/BA-6/SE/img" + ".png", "PNG")

def makesvg(x_buffer, y_buffer, c_buffer, length):
    out = '<svg xmlns="http://www.w3.org/2000/svg" width="200" height="180" version="1.1">\n'
    polyline = ''
    comma = ''
    color = "black"
    for i in range(1, length):
        # l = ",".join([str(x_buffer[i])+","+str(y_buffer[i]) for p in l])
        if c_buffer[i] != 0:
            comma = ","
        else:
            comma = ''

        polyline += comma+str(x_buffer[i])+","+str(y_buffer[i])
        if c_buffer[i] == 0:
            print("0")
            if c_buffer[i+1] == 1:
                color = "black"
            elif c_buffer[i+1] == 2:
                color = "red"
            elif c_buffer[i+1] == 3:
                color = "green"
            elif c_buffer[i+1] == 4:
                color = "blue"

        if c_buffer[i+1] == 0:
            out += '<polyline points="'+ polyline +'" stroke="'+ color +'" stroke-width="2" fill="none" />\n'
            polyline = ""
    out += '</svg>'
    return out



#     toRead = 4000
#     rcv_buffer = b''
#     # print(rcv_buffer)
#     # print(len(rcv_buffer))
#     #reads the data
#     i = 0
#     # while i < toRead:
#     #     print(ser.read())
#     #     i +=1

# # WORKING 90X100 rgb
#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(toRead)
#     # rcv_buffer += ser.read(2000)

#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(toRead)
#     rcv_buffer += ser.read(1000)

#     new_buffer = bytearray(len(rcv_buffer))
#     # new_buffer[0::2] = rcv_buffer[1::2]
#     # new_buffer[1::2] = rcv_buffer[0::2]
#     # im = Image.frombytes("RGB", (100, 90), bytes(new_buffer), "raw", "BGR;16")
#     im = Image.frombytes("L", (100, 90), bytes(rcv_buffer), "raw")
#     nameimg ="Image"
#     im.show(title=nameimg)
#     im.save("C:/Users/41786/Desktop/Projects/BA-6/SE/img" + ".png", "PNG")

def parse_arg():
    port = SERIAL_PORT_DEFAULT
    if len(sys.argv) == 1:
        print("Using default port " + port)
    elif len(sys.argv) == 2:
        port = sys.argv[1]
        print("Using the specified port: " + port)
    else:
        print("Please specify the serial port name or leave it as empty."
        " Default serial port is " +port)
    return port

def connect_to_epuck(port):
    error_count = 0
    while error_count < ERROR_COUNT_MAX:
        print("Attempting connection to " + port + "...")

        try:
            ser = serial.Serial(port, SERIAL_BAUD_RATE, 
            timeout = TIMEOUT_PERIOD)
            print("Connection established on " + port)
            break
        except serial.SerialException:
            print("Cannot establish connection to the e-puck."
            " Retrying " + str(error_count) + "/" + str(ERROR_COUNT_MAX))
            error_count += 1
            time.sleep(TIMEOUT_PERIOD)

    if error_count == ERROR_COUNT_MAX:
        print("Connection failed on " + port +
        ". Verify that the e-puck is connected to Bluetooth or that no other"
        " device/thread is accessing the port.")
        sys.exit(0)

    # discard input and ouput buffers
    # ser.reset_input_buffer()
    # ser.reset_output_buffer()
    send_command(ser, 'R') # reset the e-puck when first connecting
    return ser

# todo: check state of e-puck: certain command should not be sent when
# the robot is in a certain state
def send_command(ser, command):
    if command not in COMMANDS:
        print("Invalid command: " + str(command))
    else:
        try:
            # ser.reset_input_buffer()
            # ser.reset_output_buffer()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(b'CMD')
            ser.write(command.encode()) # send command as bytes (utf-8)

        except serial.SerialException:
            print("Error occured when sending command. "
            "Connection to e-puck lost.")
        time.sleep(2) # wait for the e-puck to properly process the command

def get_data():
    # data_color = np.array([0, 4, 6], dtype = np.uint8)                               # rewrite
    data_color = np.array([0,3,1,5,1,1,7,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], dtype = np.uint8)                               # rewrite
    # data_pos = np.array([[430, 0]], dtype = np.uint16)
    # data_pos = np.array([[512, 0], [612, 0], [612, 50], [512, 50], [512, 0]], dtype = np.uint16) # 2 squares
    data_pos = np.array([[512, 0], [463, 0],[467, 26], [461, 49], [435, 89],[415, 138], [404, 158], [417, 164], [448, 139], [450, 104], [449, 154], [455, 187], [481, 186], [477, 163], [474, 284], [487, 294], [500, 289], [502, 268], [482, 268], [481, 263], [481, 285], [494, 285], [481, 263],[494, 269], [501, 267], [508, 188], [516, 160], [509, 188], [534, 186],[553, 152], [541, 175], [568, 157], [568, 127], [542, 59], [544, 0], [512, 0]], dtype = np.uint16)
    # data_pos = np.array([[512, 0], [562, 0],[562, 50], [512, 50], [512, 0],[512, 100], [562, 100], [562, 50], [562, 0], [512, 0]], dtype = np.uint16)
    # data_pos = np.array([[513, 0], [578, 14], [627,48 ], [657, 96], [669, 154], [657,213 ], [622, 263], [571, 297], [516, 307], [454, 296], [404, 261], [372,210 ], [360, 149], [373, 93], [407,45 ], [447, 15], [512, 0]], dtype = np.uint16) # circle
    # data_pos = np.array([[512, 0], [512, 5], [517,5 ], [517, 0], [512, 0]], dtype = np.uint16) # circle
    # data_pos = np.array([[512, 0], [570, 0], [512, 0], [560, 20], [512, 0], [550, 40],[512, 0],[540, 70],[512, 0],[530, 80],[512, 0],[520, 90],[512, 0],[512, 100],[512, 0]], dtype = np.uint16)


    # data_pos = np.array([[404, 158], [417, 164],[512, 0]], dtype = np.uint16)
    # data_pos = np.array([[400, 0],[400, 200], [200, 200], [200, 0], [400, 0]], dtype = np.uint16)
    # maybe verify that arrays are of the same size
    return data_color, data_pos

def send_move_data(ser):
    data_color, data_pos = get_data()
    size = np.array([len(data_pos)], dtype=np.uint16)

    send_buffer = bytearray([])
    i = 0
    while(i < size[0]):
        
        # convert as unsigned char, integer, 1 byte
        send_buffer += struct.pack('B', data_color[i])     # color
        # convert as unsigned short int, 2 bytes
        send_buffer += struct.pack('<H', data_pos[i][0])    # x
        send_buffer += struct.pack('<H', data_pos[i][1])    # y
        i = i+1
    
    print("Sending move data to the e-puck...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b'MOVE')
    ser.write(struct.pack('<h',size[0]))

    ser.write(send_buffer)
    print("Move data sent.")



        
# ===========================================================================
#  Functions.                                                         
# ===========================================================================

# Main function =============================================================

def main():
    port = parse_arg()
    ser = connect_to_epuck(port)

    while True:
        command = input("Type a command")
        send_command(ser, command)
        if command == 'G':
            send_move_data(ser)
        if command == 'B':
            receive_image(ser)
if __name__=="__main__":
    main()
    
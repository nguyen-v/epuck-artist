# @file     epuck-communication.py
# @brief    E-puck 2 communication to computer (computer side).
# @note     We work in little endian because chSequentialStreamWrite from 
#           ChibiOS sends data in little endian when working with multibyte data

import sys
import serial
import time
import numpy as np
import struct
import threading

from PIL import Image # read image

# ===========================================================================
#  Module constants.                                                         
# ===========================================================================

# Serial 
SERIAL_PORT_DEFAULT         = "COM8"  
SERIAL_BAUD_RATE            = 115200
BT_BAUD_RATE                = 9600
MAX_BUFFER_LENGTH           = 4000

# Error, timeout
TIMEOUT_PERIOD              = 1
ERROR_COUNT_MAX             = 10
ARDUINO_ERR_COUNT_MAX       = 50

# Periods
READ_PERIOD                 = 0.1
WRITE_PERIOD                = 0.1

# Messages
CONFIRMATION_MSG             = 'Ready'


# Commands
COMMANDS = (
    'R'     ,   # RESET
    'P'     ,   # PAUSE
    'C'     ,   # CONTINUE
    'S'     ,   # SIGNAL COLOR CHANGE
    'B'     ,   # BEGIN CALIBRATION
    'G'     ,   # GET DATA
    'D'     ,   # DRAW
    'I'     ,   # INTERACTIVE
    'H'     ,   # HOME
    'V'     ,   # VALIDATE
)

# associate an index to each command
CMD_INDEX = {
    'R' : 0    ,
    'P' : 1    ,   
    'C' : 2    ,  
    'S' : 3    ,     
    'B' : 4    ,   
    'G' : 5    ,   
    'D' : 6    ,   
    'I' : 7    ,   
    'H' : 8    , 
    'V' : 9    
}

CMD_HEADER = [b'' for x in range(len(COMMANDS))]
CMD_HEADER[CMD_INDEX['V']] = b'LEN'
CMD_HEADER[CMD_INDEX['G']] = b'MOVE'

# commands that need a second argument
COMMANDS_TWO_ARGS = (
    'V'     ,   # VALIDATE
)

# associate a command to an index in the SECOND_ARG_LIMIT matrix
CMD_TWO_ARGS_INDEX = {
    'V' : 0
}

# create a matrix of size len(COMMANDS_TWO_ARG) x 2
# that will contain the lower and upper bounds of second argument
SECOND_ARG_LIMIT = [0 for x in range(2) for y in range(len(COMMANDS_TWO_ARGS))]

# assign lower and upper bounds
SECOND_ARG_LIMIT[CMD_TWO_ARGS_INDEX['V']] = [0, 150] # in mm

def receive_data(ser_epuck, ser_arduino):
    while True:
        # state machine for proper synchronisation
        state = 0
        while(state != 5):
            #reads 1 byte
            c1 = ser_epuck.read(1)

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
        print("Receiving message...")
        # Read message type and length
        msg = ser_epuck.readline().decode("utf_8")
        length = struct.unpack('<h',ser_epuck.read(2))  # length is sent as an uint16
        length = length[0]
        output_buffer = b''
        # Read rest of buffer
        if length > MAX_BUFFER_LENGTH:
            while length > MAX_BUFFER_LENGTH:
                output_buffer += ser_epuck.read(MAX_BUFFER_LENGTH)
                length -= MAX_BUFFER_LENGTH
        else:
            output_buffer = ser_epuck.read(length)

        if "color" in msg:
            print("Requesting color change " + output_buffer.decode("utf8"))
            ser_arduino.write(output_buffer)
            conf_msg = ""
            error_count = 0
            while CONFIRMATION_MSG not in conf_msg and error_count < ARDUINO_ERR_COUNT_MAX:
                conf_msg = ser_arduino.readline().decode("utf_8")
                error_count += 1
                if CONFIRMATION_MSG in conf_msg or error_count == ARDUINO_ERR_COUNT_MAX:
                    time.sleep(0.2)
                    print("Arduino has finished changing colors.")
                    send_command(ser_epuck, "S")
        time.sleep(0.1)






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




    toRead = 4000
    rcv_buffer = b''
    # print(rcv_buffer)
    # print(len(rcv_buffer))
    #reads the data
    i = 0
    # while i < toRead:
    #     print(ser.read())
    #     i +=1

# WORKING 90X100 rgb
    # rcv_buffer += ser.read(toRead)
    # rcv_buffer += ser.read(toRead)
    # rcv_buffer += ser.read(toRead)
    # rcv_buffer += ser.read(toRead)
    # rcv_buffer += ser.read(2000)

    rcv_buffer += ser.read(toRead)
    rcv_buffer += ser.read(toRead)
    rcv_buffer += ser.read(1000)

    new_buffer = bytearray(len(rcv_buffer))
    # new_buffer[0::2] = rcv_buffer[1::2]
    # new_buffer[1::2] = rcv_buffer[0::2]
    # im = Image.frombytes("RGB", (100, 90), bytes(new_buffer), "raw", "BGR;16")
    im = Image.frombytes("L", (100, 90), bytes(rcv_buffer), "raw")
    nameimg ="Image"
    im.show(title=nameimg)
    im.save("C:/Users/41786/Desktop/Projects/BA-6/SE/img" + ".png", "PNG")

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

def connect_to_port(port, baud_rate, reset):
    error_count = 0
    while error_count < ERROR_COUNT_MAX:
        print("Attempting connection to " + port + "...")

        try:
            ser = serial.Serial(port, baud_rate, 
            timeout = TIMEOUT_PERIOD)
            print("Connection established on " + port)
            break
        except serial.SerialException:
            print("Cannot establish connection to port " + port + ". " +
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
    if reset == True:
        send_command(ser, 'R') # reset the e-puck when first connecting
    return ser

# todo: check state of e-puck: certain command should not be sent when
# the robot is in a certain state
def send_command(ser, command):
    if command not in COMMANDS:
        print("Invalid command: " + str(command))
    else:
        cmd_header = CMD_HEADER[CMD_INDEX[command]]
        second_arg = b''
        if command in COMMANDS_TWO_ARGS:
            try:
                lower_bound = SECOND_ARG_LIMIT[CMD_TWO_ARGS_INDEX[command]][0]
                upper_bound = SECOND_ARG_LIMIT[CMD_TWO_ARGS_INDEX[command]][1]
                second_arg = np.uint8(input("Enter an integer between %d and %d: "
                % (lower_bound, upper_bound)))
                if (lower_bound < second_arg <  upper_bound):
                    pass
                else:
                    print("Second argument must be between %d and %d: "
                    % (lower_bound, upper_bound))
                    return
            except ValueError:
                print("Second argument must be an integer")
                return
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # send command
            ser.write(b'CMD')
            ser.write(command.encode()) # send command as bytes (utf-8)

            # send header and second argument
            ser.write(cmd_header)
            if second_arg != b'':
                ser.write(struct.pack('B', second_arg))

        except serial.SerialException:
            print("Error occured when sending command. "
            "Connection to e-puck lost.")
        time.sleep(2) # wait for the e-puck to properly process the command

        if command == 'G':
            send_move_data(ser)

def read_serial(ser):
    while True:
        msg = ser.readline().decode("utf_8")
        if CONFIRMATION_MSG in msg:
            print("Arduino has finished changing colors.")
        time.sleep(0.5)

def request_command(ser_epuck, ser_arduino):
    while True:
        command = input("Type a command: ")
        if command != 'a':
            send_command(ser_epuck, command)
            # receive_data(ser_epuck)
            # time.sleep(1)
        elif command == 'a':
            command = input("Type a command (arduino): ")
            ser_arduino.write(command.encode())

    time.sleep(0.5)

def get_data():
    # data_color = np.array([0, 4, 6], dtype = np.uint8)                               # rewrite
    data_color = np.array([0, 1, 0, 3, 4, 2], dtype = np.uint8)                               # rewrite
    # data_pos = np.array([[430, 0]], dtype = np.uint16)
    # data_pos = np.array([[512, 0], [612, 0], [612, 50], [512, 50], [512, 0]], dtype = np.uint16) # 2 squares
    # data_pos = np.array([[512, 0], [463, 0],[467, 26], [461, 49], [435, 89],[415, 138], [404, 158], [417, 164], [448, 139], [450, 104], [449, 154], [455, 187], [481, 186], [477, 163], [474, 284], [487, 294], [500, 289], [502, 268], [482, 268], [481, 263], [481, 285], [494, 285], [481, 263],[494, 269], [501, 267], [508, 188], [516, 160], [509, 188], [534, 186],[553, 152], [541, 175], [568, 157], [568, 127], [542, 59], [544, 0], [512, 0]], dtype = np.uint16)
    # data_pos = np.array([[512, 0], [562, 0],[562, 50], [512, 50], [512, 0],[512, 100], [562, 100], [562, 50], [562, 0], [512, 0]], dtype = np.uint16)
    # data_pos = np.array([[513, 0], [578, 14], [627,48 ], [657, 96], [669, 154], [657,213 ], [622, 263], [571, 297], [516, 307], [454, 296], [404, 261], [372,210 ], [360, 149], [373, 93], [407,45 ], [447, 15], [512, 0]], dtype = np.uint16) # circle
     # data_pos = np.array([[512, 0], [562, 0],[562, 50], [512, 50], [512, 0],[512, 100], [562, 100], [562, 50], [562, 0], [512, 0]], dtype = np.uint16)
    # data_pos = np.array([[512, 0], [600, 0]], dtype = np.uint16) # long line
    # data_pos = np.array([[512, 0],[512, 30], [512, 0], [612, 0],[512, 0], [412, 0], [512, 0]], dtype = np.uint16) # y test
    # data_pos = np.array([[512, 0],[512, 100], [512, 200],[412, 200], [312, 200], [312, 100],[312, 0],[412, 0],[512, 0], [512, 100], [512, 200],[612, 200], [712, 200], [712, 100],[712, 0],[612, 0],[512, 0]], dtype = np.uint16) # square test

    # data_pos = np.array([[512, 0], [512, 100], [512, 200],[412, 200], [312, 200], [312, 100],[312, 0],[412, 0],[512, 0]], dtype = np.uint16) # LEFT square test
    # data_pos = np.array([[512, 0], [512, 100], [612, 100], [612, 0], [512, 0]], dtype = np.uint16) # RIGHT square test
    # data_pos = np.array([[512, 0], [612, 0], [612, 100], [612, 200], [512, 200], [412, 200], [412,100], [412, 0], [512, 0]], dtype = np.uint16) # RIGHT square test    
    data_pos = np.array([[100, 0], [200, 0], [200, 200], [0, 200], [0, 0],[100, 0]], dtype = np.uint16) # TILTED square test    
    # data_pos = np.array([[512, 0],[512, 150]], dtype = np.uint16) # going down
    # data_pos = np.array([[480, 0]], dtype = np.uint16) # y test
    
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
    # ser.write(b'MOVE')
    ser.write(struct.pack('<h',size[0]))

    ser.write(send_buffer)
    print("Move data sent.")


        
# ===========================================================================
#  Functions.                                                         
# ===========================================================================

# Main function =============================================================

def main():
    port = parse_arg()
    ser_epuck = connect_to_port(port, SERIAL_BAUD_RATE, True)
    ser_arduino = connect_to_port("COM22", BT_BAUD_RATE, False)

    request_cmd = threading.Thread(target = request_command, args = (ser_epuck, ser_arduino))
    request_cmd.setDaemon(True)
    request_cmd.start()

    rcv_data = threading.Thread(target=receive_data, 
                                args = (ser_epuck, ser_arduino))
    rcv_data.setDaemon(True)
    rcv_data.start()

    while True:
        # command = input("Type a command: ")
        # if command != 'a':
        #     # send_command(ser, command)
        #     time.sleep(1)
        # elif command == 'a':
        #     command = input("Type a command (arduino): ")
        #     ser2.write(command.encode())
        # elif command == 'B':
        #     # receive_image(ser)
        time.sleep(0.1)
if __name__=="__main__":
    main()
    
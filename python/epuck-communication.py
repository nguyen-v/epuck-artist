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
from PIL import Image

# ========================================================================== #
#  Module constants.                                                         # 
# ========================================================================== #

# Serial 
SERIAL_PORT_DEFAULT         = "COM8"
ARDUINO_SERIAL_PORT         = "COM22"
SERIAL_BAUD_RATE            = 115200
BT_BAUD_RATE                = 9600
MAX_BUFFER_LENGTH           = 4000

# Error, timeout
TIMEOUT_PERIOD              = 1
ERROR_COUNT_MAX             = 10
ARDUINO_ERR_COUNT_MAX       = 500

# Periods
READ_PERIOD                 = 0.1
WRITE_PERIOD                = 0.1

# Messages
CONFIRMATION_MSG             = 'Ready'

# Images
IM_LENGTH_PX                = 100
IM_HEIGHT_PX                = 90
IMG_PATH                    = "C:/Users/41786/Desktop/Projects/BA-6/SE/epuck-artist/img/"

# Sobel
FIRST_OCTANT                = 1
SECOND_OCTANT               = 2
THIRD_OCTANT                = 3
FOURTH_OCTANT               = 4
FIFTH_OCTANT                = 5
SIXTH_OCTANT                = 6
SEVENTH_OCTANT              = 7
EIGHTH_OCTANT               = 8

RED    = bytes([int.from_bytes(b'\xF8', "big"), int.from_bytes(b'\x00', "big")])
ORANGE = bytes([int.from_bytes(b'\xFC', "big"), int.from_bytes(b'\xA0', "big")])
YELLOW = bytes([int.from_bytes(b'\xFF', "big"), int.from_bytes(b'\x60', "big")])
GREEN  = bytes([int.from_bytes(b'\x07', "big"), int.from_bytes(b'\xE0', "big")])
BLUE   = bytes([int.from_bytes(b'\x05', "big"), int.from_bytes(b'\x5F', "big")])
D_BLUE = bytes([int.from_bytes(b'\x01', "big"), int.from_bytes(b'\xBF', "big")])
PURPLE = bytes([int.from_bytes(b'\x88', "big"), int.from_bytes(b'\x1F', "big")])
PINK   = bytes([int.from_bytes(b'\xF8', "big"), int.from_bytes(b'\x1D', "big")])

# Commands

COMMANDS = (
    'R'     ,   # RESET
    'P'     ,   # PAUSE
    'C'     ,   # CONTINUE
    'S'     ,   # SIGNAL COLOR CHANGE
    'B'     ,   # BEGIN CALIBRATION
    'G'     ,   # GET DATA
    'D'     ,   # DRAW
    'I'     ,   # IMAGE
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

# ========================================================================== #
#  Module local functions.                                                   # 
# ========================================================================== #

# @brief                    Receives data from e-puck and communicates with Arduino.
# @param[in]   ser_epuck    E-puck serial port
# @param[out]  ser_arduino  Arduino serial port
# @return                   none 
# @note                     state machine/length extraction is from TP4, plotImage.py
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

        if "path" in msg:
            x_buffer = b''
            y_buffer = b''
            c_buffer = b''
            x_buffer = ser_epuck.read(length)
            y_buffer = ser_epuck.read(length)
            c_buffer = ser_epuck.read(length)
            c_buffer += b'\x00' # to avoid being out of range

            f = open(IMG_PATH + "path.svg",'w')
            f.write(create_svg(x_buffer, y_buffer, c_buffer, length))
            f.close()

        else:
            output_buffer = b''
            # Read rest of buffer
            length_to_read = length
            if length_to_read > MAX_BUFFER_LENGTH:
                while length_to_read > MAX_BUFFER_LENGTH:
                    output_buffer += ser_epuck.read(MAX_BUFFER_LENGTH)
                    length_to_read -= MAX_BUFFER_LENGTH
            output_buffer += ser_epuck.read(length_to_read)
            img_name = ''

            if "color" in msg:
                print("Requesting color change " + output_buffer.decode("utf8"))
                ser_arduino.reset_input_buffer()
                ser_arduino.reset_output_buffer()
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
            elif "rgb" in msg:
                img_buffer = bytearray(len(output_buffer))
                # Invert bytes to make it readable for BGR;16 format
                img_buffer[0::2] = output_buffer[1::2] 
                img_buffer[1::2] = output_buffer[0::2]
                img = Image.frombytes("RGB", (IM_LENGTH_PX, IM_HEIGHT_PX), bytes(img_buffer), "raw", "BGR;16")
                img_name = "rgb"

            elif "grayscale" in msg or "gauss" in msg or "local" in msg or "canny" in msg:
                img = Image.frombytes("L", (IM_LENGTH_PX, IM_HEIGHT_PX), bytes(output_buffer), "raw")
                if "grayscale" in msg:
                    img_name = "grayscale"
                elif "gauss" in msg:
                    img_name = "gauss"
                elif "local" in msg:
                    img_name = "local_max"
                elif "canny" in msg:
                    img_name = "canny"

            elif "sobel" in msg:
                img_buffer = bytearray(2*len(output_buffer))
                for i in range (0, length-1):
                    # convert angle states to rgb565 colors
                    rgb = bytes([0, 0])
                    if output_buffer[i] == FIRST_OCTANT:
                        rgb = RED
                    elif output_buffer[i] == SECOND_OCTANT:
                        rgb = ORANGE
                    elif output_buffer[i] == THIRD_OCTANT:
                        rgb = YELLOW
                    elif output_buffer[i] == FOURTH_OCTANT:
                        rgb = GREEN
                    elif output_buffer[i] == FIFTH_OCTANT:
                        rgb = BLUE
                    elif output_buffer[i] == SIXTH_OCTANT:
                        rgb = D_BLUE
                    elif output_buffer[i] == SEVENTH_OCTANT:
                        rgb = PURPLE
                    elif output_buffer[i] == EIGHTH_OCTANT:
                        rgb = PINK

                    img_buffer[2*i] = rgb[1]
                    img_buffer[2*i+1] = rgb[0]
                img = Image.frombytes("RGB", (IM_LENGTH_PX, IM_HEIGHT_PX), bytes(img_buffer), "raw", "BGR;16")
                img_name = "sobel"

            if ("color" not in msg):
                img.save(IMG_PATH + img_name + ".png", "PNG")
        time.sleep(0.5)

# @brief                    Creates svg file from x, y and color buffers
# @param[in]   x_buffer     Path x-coordinate buffer
# @param[in]   y_buffer     Path y-coordinate buffer
# @param[in]   c_buffer     Path color buffer
# @param[in]   length       Length of each buffer
# @return      out          String containing path information in svg format
def create_svg(x_buffer, y_buffer, c_buffer, length):
    out = '<svg xmlns="http://www.w3.org/2000/svg" width="200" height="180" version="1.1">\n'
    polyline = ''
    comma = ''
    color = "black"
    for i in range(1, length):
        if c_buffer[i] != 0:
            comma = ","
        else:
            comma = ''

        polyline += comma+str(x_buffer[i])+","+str(y_buffer[i])
        if c_buffer[i] == 0:
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

# @brief                    Parses argument from command line (port name)
# @return      port         Port name
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

# @brief                    Attemps serial connection to specified port
# @param[in]   port         Port name to attempt connection to
# @param[in]   baud_rate    Serial connection baud rate
# @param[in]   reset        True if reset command should be sent on connection
# @return      ser          Serial connection to port
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

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    if reset == True:
        send_command(ser, 'R') # reset the e-puck when first connecting
    return ser

# @brief                    Creates svg file from x, y and color buffers
# @param[in]   x_buffer     Path x-coordinate buffer
# @param[in]   y_buffer     Path y-coordinate buffer
# @param[in]   c_buffer     Path color buffer
# @param[in]   length       Length of each buffer
# @return      out          String containing path information in svg format
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

# @brief                    Parses command from user
# @param[in]   ser_epuck    E-puck serial port
# @param[out]  ser_arduino  Arduino serial port
# @return                   none
def request_command(ser_epuck, ser_arduino):
    while True:
        command = input("Type a command: ")
        if command != 'a':
            send_command(ser_epuck, command)
        elif command == 'a':
            command = input("Type a command (arduino): ")
            ser_arduino.write(command.encode())

    time.sleep(0.5)

# @brief                    Returns color and position buffers manually defined
# @return      data_color   Buffer containing color data
# @return      data_pos     Buffer containing position data
def get_data():

    # Test data to send can be defined here
    data_color = np.array([0, 1, 1, 1, 1, 1], dtype = np.uint8) 
    data_pos = np.array([[100, 0], [150, 0], [150, 150], [0, 150], [0, 0],[100, 0]], dtype = np.uint16) # square test    

    return data_color, data_pos

# @brief                    Manually send color and path data to e-puck
# @param[in]   ser          Output port
# @return                   none
def send_move_data(ser):
    data_color, data_pos = get_data()
    size = np.array([len(data_pos)], dtype=np.uint16)

    send_buffer = bytearray([])
    i = 0
    while(i < size[0]):
        
        # convert as unsigned char, integer, 1 byte
        send_buffer += struct.pack('B', data_color[i])      # color
        # convert as unsigned short int, 2 bytes
        send_buffer += struct.pack('<H', data_pos[i][0])    # x
        send_buffer += struct.pack('<H', data_pos[i][1])    # y
        i = i+1
    
    print("Sending move data to the e-puck...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(struct.pack('<h',size[0]))

    ser.write(send_buffer)
    print("Move data sent.")


        
# ========================================================================== #
#  Main function.                                                            # 
# ========================================================================== #

# @brief                    Main function. Initializes threads.
# @return                   none
def main():
    port = parse_arg()
    ser_epuck = connect_to_port(port, SERIAL_BAUD_RATE, True)
    ser_arduino = connect_to_port(ARDUINO_SERIAL_PORT, BT_BAUD_RATE, False)

    request_cmd = threading.Thread(target = request_command, args = (ser_epuck, ser_arduino))
    request_cmd.setDaemon(True)
    request_cmd.start()

    rcv_data = threading.Thread(target=receive_data, 
                                args = (ser_epuck, ser_arduino))
    rcv_data.setDaemon(True)
    rcv_data.start()
    while True:
         time.sleep(0.1)
if __name__=="__main__":
    main()
    
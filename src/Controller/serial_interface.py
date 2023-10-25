import serial
import numpy as np
import serial.tools.list_ports as ports
import os
import time
import yaml
import struct
import logging

# ---------------------------- display and select available ports ----------------------------
def choose_port():
    portsList = []

    for onePort in ports.comports():
        portsList.append(str(onePort))
        logging.info(str(onePort))
    val = input("Select port: COM")

    for x in range(0, len(portsList)):
        if portsList[x].startswith("COM" + str(val)):
            portvar = "COM" + str(val)  # open serial port
            logging.info("Port selected: " + portvar)         # check which port was really used

    return portvar


# ---------------------------- full state pid control ----------------------------
def pid(z,K,zr):
    # two controller stages
    # match
        # case swing up
            # Lyapunov based

        # case near top
            # calculate error z - r
            # calculate feedback u = -K*e
            # saturation
    
    u = -np.sum(np.multiply(z-zr,K))
    # return u
    return u

# ---------------------------- send control commands ----------------------------
def cmd_motor(serialInst, u):
    
    # saturation
    if abs(u) > 100: u = np.sign(u)*100
    
    # prompt primary board
    serialInst.write(b'5')

    # send cmd
    serialInst.write(u.to_bytes(1, "little"))

# ---------------------------- process state feedback ----------------------------
def read_state(serialInst, error, timeout):
    # check if there's error
    while serialInst.in_waiting: 
        rcode = int.from_bytes(serialInst.read(), 'little')
        try:
            match rcode >> 5:
                case 1:
                    logging.warning("IMU error: " + error[rcode - 0b1 << 5])
                case 2:
                    logging.warning("Cart error: " + error[rcode - 0b1 << 6])
                case 4:
                    logging.warning("Axis error: " + error[rcode - 0b1 << 7])
        except:
            logging.warning("Unknown Error")

    # prompt primary board to send sensor data
    serialInst.flush()
    serialInst.write(b'2')

    # wait for data to be ready
    start = time.time()
    while (info := serialInst.read()) != b'3' and time.time()-start < timeout:
        pass

    # wait until all data are transmitted
    start = time.time()
    while (serialInst.in_waiting != 10) and time.time()-start < timeout:
        pass
    
    # if data is missing, raise a warning
    if serialInst.in_waiting < 10: 
        logging.warning("Read error during serial transmission")
        return -1

    data = serialInst.read(10)    # read data (10 byte struct)

    # unpack each system states (little endian)
    try:
        x = (600*0.05/np.pi)*int.from_bytes(data[0:2], 'little')         # extract cart position (int16_t)  
        dx = int.from_bytes(data[2:4], 'little')        # extract cart velocity (int16_t)  
        omega = int.from_bytes(data[4:6], 'little')        # extract pole angular rate (int16_t)  
        theta = (struct.unpack('<f', data[6:10]))[0]    # extract pole angle (4 byte float)  
    except:
        logging.warn("Recieved invalid data")
        return -1

    # convert cart encoder feedback (sec/tick -> rad/sec)
    dir = (omega >> 14) & 0b01
    idle = omega >> 15
    omega = 0 if idle else (600/np.pi)/((omega & ~(0b11 << 14)) if dir else (-omega & ~(0b11 << 14)))
  
    # complementary/ Kalman filter to fuse IMU and encoder 
    # compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    
    # process x axis encoder
    dir = (dx >> 14) & 0b01
    idle = dx >> 15
    dx = 0 if idle else (600/np.pi)/((dx & ~(0b11 << 14)) if dir else (-dx & ~(0b11 << 14)))
  
    # filter x encoder data by applying filter with motor output
    # => get x and dx
    
    z = np.array([x, dx, omega, theta])
    logging.info(z)   
 
    # return full state z = [x theta x' theta']
    return 0


# ---------------------------- initialize MCUs and configure global variables ----------------------------
def initialize(serialInst, error):
    # TODO:
    #   - add different controller options
    #   - other parameters, control bandwidth, motor saturation, etc
    #   - set up nano

    # start and perform synchronization
    input("Press enter to start")
    serialInst.write(b'1')

    # first initialize primary board
    while (info := serialInst.read()) != b'1':
        if info == b'':
            logging.warning("Waiting for primary board to start, will try every second")
            serialInst.write(b'1')
        else:
            raise Exception("Previous program not terminated properly, restart Arduino then retry")
        time.sleep(1)
    logging.info("primary board initialized...")
        
    # wait until other boards are initialized
    start_order = ("cart", "x axis", "IMU")
    for i in range(1):
        while (info := serialInst.read()) != b'1':
            if info == b'':
                logging.warning("Waiting for " + start_order[i] + " to start")
            else:
                try:
                    logging.error(start_order[i] + " error: " + error[int.from_bytes(info, 'little') - int(0b1 << (7-i))])
                except:
                    logging.error(start_order[i] + " unknown error")

        logging.info(start_order[i] + " connected...")
    
    # wait for final prompt after MCU variable initialization
    while (info := serialInst.read()) != b'2':  
        logging.info("motor initializing")
    
    # start program
    logging.info("program starting...")
    serialInst.timeout = 0.1

# ---------------------------- main ----------------------------
if __name__ == "__main__":
    # TODO:
    #   - make this into a ros node
    #   - multithreading
    #   - GUI
    #   - reset function in encoder.cpp to reset encoder counter based on read
    
    # import parameters and error log from yaml file as a dictionary 
    with open('config.yaml', 'r') as file:
        dict_ = yaml.safe_load(file)
    
    pid = list(dict_['pid'].values())
    K = np.array([pid[0], pid[3], pid[2], pid[5]])
    looprate = dict_['looprate']
    zr = np.multiply(np.array(dict_['zr']), np.array([1, np.pi/180, 1, 1]))
    
    # configure logging 
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)

    # initialize serial comm
    serialInst = serial.Serial()   
    serialInst.timeout = 1.0    # timeout for initialization
    serialInst.port = "COM" + str(dict_['port'])
    # serialInst.port = choose_port()   # used for debugging port issues
    serialInst.baudrate = 115200
    serialInst.open()

    # initialize hardware
    initialize(serialInst, dict_['error'])

    # indefinite control loop
    while True:
        # typical control pipeline
        try: 
            start = time.time()
            # get feedback
            z = read_state(serialInst, dict_['error'], dict_['timeout'])
            # control logic
            u = pid(z,K,zr)
            # actuate
            cmd_motor(serialInst, u)
            # regulate loop rate
            if time.time() - start > 1/looprate: 
                logging.warning("Experiencing long latency")
            while time.time() - start < 1/looprate:
                pass          
            # GUI in separate thread?

        # in case of keyboard interrupt
        except KeyboardInterrupt:
            logging.info("Program terminating ...")
            # send to MCU prompt to restart
            serialInst.write(b'4')
            while (info := serialInst.read()) != b'5':
                if info == b'':
                    logging.info("Waiting for boards to reset")
                serialInst.write(b'4')
            time.sleep(1)

            logging.info("Done!")
            exit()
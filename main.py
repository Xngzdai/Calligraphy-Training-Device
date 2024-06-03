import numpy as np
from numpy.typing import ArrayLike
import os
import serial
import socket
import sys
import signal
import time

#------------ Constants --------------#
CUR_PATH = os.getcwd()

# Canvas
MAP_LIST = {"beng": "beng.npy",
            "dian": "dian.npy", 
            "ji2": "ji2.npy", 
            "ji4": "ji4.npy", 
            "xiao": "xiao.npy", 
            "ying": "ying.npy", 
            "liu": "liu.npy"}
choice = "liu"                          # TODO: user choice

# Arduino
BAUDRATE = 115200                       # TODO: to confirm with the arduino code
PORT_L = '/dev/cu.usbserial-AQ02OE6B'   # TODO: Check the true port address
PORT_R = '/dev/cu.usbserial-AQ02O6ZP'   # TODO: Check the true port address
TIMEOUT = 1 # [sec]

# Processing
S_ADDR = '127.0.0.1'
PORT_P = 327     # Reserve a port on the computer


#------------- function definition -------------#

def signal_handler(sig, frame):
    print("\n Keyboard Interrupt Exit.")
    s.close()
    sys.exit(0)

def snap_to_grid(world_pos: ArrayLike, workspace_min=np.array([-0.04, -0.04]),
                 workspace_max=np.array([0.04,0.04]), mapspace=np.array([224,224])) -> np.array:
    """_summary_

    Args:
        world_pos (ArrayLike): _description_
        workspace_min (_type_, optional): _description_. Defaults to np.array([-1, -1]).
        workspace_max (_type_, optional): _description_. Defaults to np.array([1,1]).
        mapspace (_type_, optional): _description_. Defaults to np.array([225,225]).

    Returns:
        np.array: _description_
    """
    pos = np.clip(world_pos, workspace_min, workspace_max)
    pos_px = mapspace * (pos - workspace_min) / (workspace_max - workspace_min)
    return pos_px.astype(np.uint32)

def send_arduino(port: serial.Serial, data: float) -> bool:
    """_summary_

    Args:
        port (serial.Serial): _description_
        data (float): _description_

    Returns:
        bool: _description_
    """
    ret = True
    try:
        port.write(str(data).encode('ascii'))
    except:
        ret = False
    return ret

def recv_arduino(port: serial.Serial) -> float:
    """_summary_

    Args:
        port (serial.Serial): _description_

    Returns:
        float: _description_
    """
    data = port.readline().decode('ascii')
    if data != "":
        return float(data)
    return 0.

def send_processing(conn: socket, map: str, data: ArrayLike) -> bool:
    """_summary_

    Args:
        conn (socket): _description_
        data (ArrayLike): _description_

    Returns:
        bool: _description_
    """
    ret = True
    try:
        msg = map + " " + str(data[0]) + " " + str(data[1])
        conn.sendall(str.encode(msg + "\n"))   # Send a message to the client
    except:
        ret = False
    return ret

def norm(v1_x, v1_y, v2_x, v2_y):
    return np.sqrt((v1_x - v2_x) ** 2 + (v1_y - v2_y) ** 2)

def Jacobian(L1, L2, L3, L4, t1, t5, x2, y2, x3, y3, x4, y4, xH, yH):
    d = norm(x2, y2, x4, y4)
    b = norm(x2, y2, xH, yH)
    h = norm(x3, y3, xH, yH)
    
    del1_x2 = -L1 * np.sin(t1)
    del1_y2 = L1 * np.cos(t1)
    del1_x4 = 0
    del1_y4 = 0
    del5_x2 = 0
    del5_y2 = 0
    del5_x4 = -L4 * np.sin(t5)
    del5_y4 = L4 * np.cos(t5)
    
    # Joint 1
    del1_d = (((x4 - x2) * (-del1_x2)) + ((y4 - y2) * (-del1_y2))) / d
    del1_b = del1_d - (del1_d * (((L2 * L2) - (L3 * L3) + (d * d)) / (2.0 * d * d)))
    del1_h = -b * del1_b / h
    
    del1_yh = del1_y2 + (del1_b * d - del1_d * b) / (d * d) * (y4 - y2) + b / d * (del1_y4 - del1_y2)
    del1_xh = del1_x2 + (del1_b * d - del1_d * b) / (d * d) * (x4 - x2) + b / d * (del1_x4 - del1_x2)
    
    del1_y3 = del1_yh - h / d * (-del1_x2) - (del1_h * d - del1_d * h) / (d * d) * (x4 - x2)
    del1_x3 = del1_xh + h / d * (del1_y4 - del1_y2) + (del1_h * d - del1_d * h) / (d * d) * (y4 - y2)
    
    # Joint 2
    del5_d = (((x4 - x2) * del5_x4) + ((y4 - y2) * del5_y4)) / d
    del5_b = del5_d - (del5_d * (L2 * L2 - L3 * L3 + d * d)) / (2.0 * d * d)
    del5_h = -b * del5_b / h
    
    del5_yh = del5_y2 + (del5_b * d - del5_d * b) / (d * d) * (y4 - y2) + b / d * (del5_y4 - del5_y2)
    del5_xh = del5_x2 + (del5_b * d - del5_d * b) / (d * d) * (x4 - x2) + b / d * (del5_x4 - del5_x2)
    
    del5_y3 = del5_yh - h / d * (del5_x4 - del5_x2) - (del5_h * d - del5_d * h) / (d * d) * (x4 - x2)
    del5_x3 = del5_xh + h / d * (del5_y4 - del5_y2) + (del5_h * d - del5_d * h) / (d * d) * (y4 - y2)

    return np.array([[del1_x3, del1_y3], [del5_x3, del5_y3]])

def forward_kinematics(left_angle: float, right_angle: float) -> np.array:
    """_summary_

    Args:
        left_angle (float): angle at P5 in rad
        right_angle (float): angle at P1 in rad

    Returns:
        np.array: (2,)
    """
    # ####################### PART (a): YOUR CODE BELOW #######################

    # Define lengths
    L1 = L4 = 0.127  # Length between P1 and P2, and P4 and P5
    L2 = L3 = 0.1524  # Length between P2 and P3, and P3 and P4
    L5 = 0.08509  # Length between P5 and P1
    theta = np.radians(131.5)
    theta1 = np.pi - theta - left_angle
    theta5 = theta - right_angle

    # Point P1 (x1, y1)
    x1, y1 = 0, 0

    # Point P5 (x5, y5)
    x5 = x1 - L5
    y5 = y1

    # Point P2 (x2, y2)
    x2 = x1 + L1 * np.cos(theta1)
    y2 = y1 + L1 * np.sin(theta1)

    # Point P4 (x4, y4)
    x4 = x5 + L4 * np.cos(theta5)
    y4 = y5 + L4 * np.sin(theta5)

    # Distance ||P4 - P2||
    d_P4_P2 = np.sqrt((x4 - x2)**2 + (y4 - y2)**2)

    # Angle α using Law of Cosines
    cos_alpha = (L2**2 + d_P4_P2**2 - L3**2) / (2 * L2 * d_P4_P2)
    # alpha = np.arccos(cos_alpha)
    d_PH_P2 = L2 * cos_alpha

    # Position of Point P_H (Ph_x, Ph_y)
    Ph_x = x2 + (d_PH_P2 / d_P4_P2) * (x4 - x2)
    Ph_y = y2 + (d_PH_P2 / d_P4_P2) * (y4 - y2)

    # Distance ||P3 - P_H||
    d_P3_PH = np.sqrt(L2**2 - d_PH_P2**2)

    # Position of Point P3 (x3, y3)
    x3 = Ph_x + (d_P3_PH / d_P4_P2) * (y4 - y2)
    y3 = Ph_y - (d_P3_PH / d_P4_P2) * (x4 - x2)

    x3 = -x3
    y3 -= 0.22

    Jv = Jacobian(L1, L2, L3, L4, theta1, theta5, x2, y2, x3, y3, x4, y4, Ph_x, Ph_y)
    

    return np.array([x3, y3]), Jv
    

    # ############################# END PART (a) ##############################

def compute_force(gradient: np.array, left_angle: float, right_angle: float) -> np.array:
    """_summary_

    Args:
        gradient (np.array): (2,) x, y gradient force

    Returns:
        np.array: (2,) Left force and Right force
    """
    # ####################### PART (b): YOUR CODE BELOW #######################
    
    # Calculate positions using forward kinematics
    _, Jv = forward_kinematics(left_angle, right_angle)
    torque = Jv.T @ gradient
    return torque

    # ############################# END PART (b) ##############################

#------------ Load Canvas --------------#
map = np.load(os.path.join(CUR_PATH, "map", MAP_LIST[choice]))
assert np.all(map.shape == (2, 225, 225))

#---------- memory variables -----------#
# TODO: Can implement a temporal difference filter for smoothness
world_pos = np.array([0, 0], dtype=np.float32)
pixel_pos = np.array([0, 0], dtype=np.uint32)
force = np.array([0, 0], dtype=np.float32)

#---------- Left Arduino port setup -----------#
portArduinoLeft = serial.Serial()
print("Open port at: ", portArduinoLeft.name)
portArduinoLeft.baudrate = BAUDRATE
portArduinoLeft.port = PORT_L
portArduinoLeft.timeout = TIMEOUT

#---------- Right Arduino port setup -----------#
portArduinoRight = serial.Serial()
print("Open port at: ", portArduinoRight.name)
portArduinoRight.baudrate = BAUDRATE
portArduinoRight.port = PORT_R
portArduinoRight.timeout = TIMEOUT

#---------- Processing socket setup -----------#
s = socket.socket()
print("Socket successfully created")
s.bind((S_ADDR, PORT_P))         
print("Socket binded to %s" %(PORT_P) )
s.listen(5)    # Put the socket into listening mode       
print("Socket is listening"       )

#---------- Connection -----------#
portArduinoLeft.open()
portArduinoRight.open()
c, addr = s.accept()   # Establish connection with client

with c:
    print('Got Processing connection from', addr)
    #---------- Control Loop -----------#
    while True:
        # Visualization
        send_processing(c, choice, pixel_pos)

        # Listen from Arduino
        if not portArduinoLeft.is_open:
            portArduinoLeft.open()
            continue
        if not portArduinoRight.is_open:
            portArduinoRight.open()
            continue
        left_angle = np.radians(recv_arduino(portArduinoLeft))
        right_angle = np.radians(recv_arduino(portArduinoRight))

        print("angles: ", left_angle, right_angle)

        # Transform to world position
        # changed
        world_pos, _ = forward_kinematics(left_angle, right_angle)

        # Transform to pixel position
        print("world pos: ", world_pos)
        pixel_pos = snap_to_grid(world_pos)
        print("pixel pos: ", pixel_pos)

        # Find the gradient
        grad = map[:, pixel_pos[0], pixel_pos[1]] / 25
        # print(pixel_pos, map.shape, grad.shape)

        # Calculate the force array
        torque = compute_force(gradient=grad, left_angle=left_angle, right_angle=right_angle)

        # Transimit force message to arduino
        send_arduino(portArduinoLeft, torque[0])
        send_arduino(portArduinoRight, torque[1])

        signal.signal(signal.SIGINT, signal_handler)

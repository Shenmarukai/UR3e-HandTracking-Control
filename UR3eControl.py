#!/usr/bin/env python

import math
import socket
import numpy as np
import quaternion
import rtde_control
import rtde_receive

# Master UR Robot Enable [True, False]
Right_Master_Robot_Enabled = True
Left_Master_Robot_Enabled = False

# UR Robot Gripper Enable [True, False]
Right_Gripper_Enabled = True
Left_Gripper_Enabled = False

# End Effector Rotation Enable/Disable [True, False]
Right_EERotations = False
Left_EERotations = False

# Enable/Disable Forward Kinematics [True, False]
Right_Kinematics_Enabled = False
Left_Kinematics_Enabled = False

# UR Robot IP-Addresses
UR_IP_R = "192.168.1.1"
UR_IP_L = "192.168.1.2"

# RTDE Control/Receive
if Right_Master_Robot_Enabled == True:
    rtde_c_r = rtde_control.RTDEControlInterface(UR_IP_R)
    rtde_r_r = rtde_receive.RTDEReceiveInterface(UR_IP_R)
if Left_Master_Robot_Enabled == True:
    rtde_c_l = rtde_control.RTDEControlInterface(UR_IP_L)
    rtde_r_l = rtde_receive.RTDEReceiveInterface(UR_IP_L)

# Gripper Port
PORTGRIP = 63352

# PC IP-Address
HOST = "192.168.1.1"  # Standard loopback interface address (localhost)
PORT = 2048  # Port to listen on (non-privileged ports are > 1023)

# Control Type [Absolute, Relative]
Control = "Absolute"

# PID Position Gain
PID_Follow_Strength = 100

# Robotic Link Length Array
link = np.array([0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921])

# UR Robot Enable/Disable [True, False]
Right_Robot_Enabled = False
Left_Robot_Enabled = False

# Position:
right_px = 0
right_py = 0
right_pz = 0
left_px = 0
left_py = 0
left_pz = 0

# Rotation:
right_u = 0
right_v = 0
right_w = 0
left_u = 0
left_v = 0
left_w = 0

# Velocity:
right_vx = 0
right_vy = 0
right_vz = 0
left_vx = 0
left_vy = 0
left_vz = 0

# End Effector Radius from Center
right_r_xy = 0
right_r_z = 0
left_r_xy = 0
left_r_z = 0

# End Effector Angle from Center
right_r_xy_angle = 0
right_r_z_angle = 0
left_r_xy_angle = 0
left_r_z_angle = 0

# Offsets:
right_x_offset = 0.5
right_y_offset = -0.15
right_z_offset = 0
left_x_offset = 0.5
left_y_offset = -0.15
left_z_offset = 0

# Workspace
Workspace_Outer = 0.5
Workspace_Inner = 0.25
Workspace_Base = 0.09

# Read in Bytes Function
def readnbyte(sock, n):
    buff = bytearray(n)
    pos = 0
    while pos < n:
        cr = sock.recv_into(memoryview(buff)[pos:])
        if cr == 0:
            raise EOFError
        pos += cr
    return buff

# Forward Kinematics
def ControlCommand(Kinematics, link, q_actual):
    if Kinematics == True:

        # Point Vector from frame 0 to frame 1:
        x_1 = 0
        y_1 = -link[0]
        z_1 = 0

        # Point Vector from frame 0 to frame 2:
        x_2 = link[1] * math.cos(q_actual[0])
        y_2 = -link[0]
        z_2 = link[1] * math.sin(q_actual[0])

        # Point Vector from frame 0 to frame 3:
        x_3 = link[2] * math.cos(q_actual[0] + q_actual[1]) + link[1] * math.cos(q_actual[0])
        y_3 = -link[0]
        z_3 = link[2] * math.sin(q_actual[0] + q_actual[1]) + link[1] * math.sin(q_actual[0])

        # Point Vector from frame 0 to frame 4:
        x_4 = link[2] * math.cos(q_actual[0] + q_actual[1]) + link[1] * math.cos(q_actual[0]) + link[3] * math.sin(q_actual[0] + q_actual[1] + q_actual[2])
        y_4 = -link[0]
        z_4 = link[2] * math.sin(q_actual[0] + q_actual[1]) + link[1] * math.sin(q_actual[0]) - link[3] * math.cos(q_actual[0] + q_actual[1] + q_actual[2])

        # Point Vector from frame 0 to frame 5:
        x_5 = link[2] * math.cos(q_actual[0] + q_actual[1]) + link[1] * math.cos(q_actual[0]) + link[3] * (math.cos(q_actual[0] + q_actual[1]) * math.sin(q_actual[2]) + math.sin(q_actual[0] + q_actual[1]) * math.cos(q_actual[2])) + link[4] * math.sin(q_actual[3]) * (math.sin(q_actual[0] + q_actual[1]) * math.sin(q_actual[2]) - math.cos(q_actual[0] + q_actual[1]) * math.cos(q_actual[2]))
        y_5 = -link[0] - link[4] * math.cos(q_actual[3])
        z_5 = link[2] * math.sin(q_actual[0] + q_actual[1]) + link[1] * math.sin(q_actual[0]) + link[3] * (math.sin(q_actual[0] + q_actual[1]) * math.sin(q_actual[2]) - math.cos(q_actual[0] + q_actual[1]) * math.cos(q_actual[2])) - link[4] * math.sin(q_actual[3]) * (math.cos(q_actual[0] + q_actual[1]) * math.sin(q_actual[2]) + math.sin(q_actual[0] + q_actual[1]) * math.cos(q_actual[2]))

        # Point Vector from frame 0 to frame 6:
        x_6 = link[2] * math.cos(q_actual[0] + q_actual[1]) + link[1] * math.cos(q_actual[0]) + link[3] * math.sin(q_actual[0] + q_actual[1] + q_actual[2]) - link[4] * math.cos(q_actual[0] + q_actual[1] + q_actual[2]) * math.sin(q_actual[3]) - link[5] * math.cos(q_actual[0] + q_actual[1] + q_actual[2]) * math.sin(q_actual[3])
        y_6 = -link[0] - link[4] * math.cos(q_actual[3]) - link[5] * math.cos(q_actual[3])
        z_6 = link[2] * math.sin(q_actual[0] + q_actual[1]) + link[1] * math.sin(q_actual[0]) - link[3] * math.cos(q_actual[0] + q_actual[1] + q_actual[2]) - link[4] * math.sin(q_actual[0] + q_actual[1] + q_actual[2]) * math.sin(q_actual[3]) - link[5] * math.sin(q_actual[0] + q_actual[1] + q_actual[2]) * math.sin(q_actual[3])

        LinkPositions = np.array([x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_4, y_4, z_4, x_5, y_5, z_5, x_6, y_6, z_6])

    return LinkPositions

# Robot Absolute Control
def ControlCommand(Robot, Control, PID_Follow_Strength, Workspace_Outer, Workspace_Inner, Workspace_Base, gripper_grab, px, py, pz, rot_x, rot_y, rot_z, r_xy, r_z, r_xy_angle, r_z_angle):
    if (Control == "Absolute") and (gripper_grab < 255):
        if (r_z < Workspace_Outer) and (r_xy > Workspace_Inner) and (py > Workspace_Base):
            Robot.servoL([px, pz, py, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
        if (r_z < Workspace_Outer) and (r_xy > Workspace_Inner) and (py < Workspace_Base):
            Robot.servoL([px, pz, Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
        if (px > 0) and (pz < 0):
            if (py > Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), -abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), abs(Workspace_Outer * math.sin(r_z_angle)), rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([abs(Workspace_Inner * math.cos(r_xy_angle)), -abs(Workspace_Inner * math.sin(r_xy_angle)), py, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
            if (py < Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), -abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([abs(Workspace_Inner * math.cos(r_xy_angle)), -abs(Workspace_Inner * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
        if (px < 0) and (pz < 0):
            if (py > Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([-abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), -abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), abs(Workspace_Outer * math.sin(r_z_angle)), rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([-abs(Workspace_Inner * math.cos(r_xy_angle)), -abs(Workspace_Inner * math.sin(r_xy_angle)), py, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
            if (py < Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([-abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), -abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([-abs(Workspace_Inner * math.cos(r_xy_angle)), -abs(Workspace_Inner * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
        if (px > 0) and (pz > 0):
            if (py > Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), abs(Workspace_Outer * math.sin(r_z_angle)), rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([abs(Workspace_Inner * math.cos(r_xy_angle)), abs(Workspace_Inner * math.sin(r_xy_angle)), py, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
            if (py < Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([abs(Workspace_Inner * math.cos(r_xy_angle)), abs(Workspace_Inner * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
        if (px < 0) and (pz > 0):
            if (py > Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([-abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), abs(Workspace_Outer * math.sin(r_z_angle)), rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([-abs(Workspace_Inner * math.cos(r_xy_angle)), abs(Workspace_Inner * math.sin(r_xy_angle)), py, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
            if (py < Workspace_Base):
                if (r_z >= Workspace_Outer) and (r_xy > Workspace_Inner):
                    Robot.servoL([-abs(Workspace_Outer * math.cos(r_z_angle) * math.cos(r_xy_angle)), abs((Workspace_Outer * math.cos(r_z_angle)) * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)
                if r_xy <= Workspace_Inner:
                    Robot.servoL([-abs(Workspace_Inner * math.cos(r_xy_angle)), abs(Workspace_Inner * math.sin(r_xy_angle)), Workspace_Base, rot_x, rot_y, rot_z], 0.0, 0.0, 0.01, 0.2, PID_Follow_Strength)

# Socket Communication Initialization
if Right_Master_Robot_Enabled == 1:
    sgr = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if Left_Master_Robot_Enabled == 1:
    sgl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()

# Right Gripper Connect
if Right_Master_Robot_Enabled == True:
    if Right_Gripper_Enabled == True:
        sgr.connect((UR_IP_R, PORTGRIP))

# Left Gripper Connect
if Left_Master_Robot_Enabled == True:
    if Left_Gripper_Enabled == True:
        sgl.connect((UR_IP_L, PORTGRIP))

# Main Program
with conn:

    print(f"Connected by {addr}")

    # Enable Right Gripper
    if Right_Master_Robot_Enabled == True:
        if Right_Gripper_Enabled == True:
            sgr.sendall(b'SET ACT 1\n')

    # Enable Left Gripper
    if Left_Master_Robot_Enabled == True:
        if Left_Gripper_Enabled == True:
            sgl.sendall(b'SET ACT 1\n')

    # Main Loop
    while True:

        # Enable Right Gripper Position Control
        if Right_Master_Robot_Enabled == True:
            if Right_Gripper_Enabled == True:
                sgr.sendall(b'SET GTO 1\n')

        # Enable Left Gripper Position Control
        if Left_Master_Robot_Enabled == True:
            if Left_Gripper_Enabled == True:
                sgl.sendall(b'SET GTO 1\n')

        # Capture Old Right Hand Position
        right_x_old = right_px
        right_y_old = right_py
        right_z_old = right_pz

        # Capture Old Left Hand Position
        left_x_old = left_px
        left_y_old = left_py
        left_z_old = left_pz

        # Read In Leap Motion Data
        data = readnbyte(conn, 84)

        # Leap Motion Data Collection
        # Right Hand X Position:
        if data[0] == 0:
            right_px = -((65026 * data[1]) + (256 * data[2]) + data[3]) / 10000 + right_x_offset
        if data[0] == 1:
            right_px = ((65026 * data[1]) + (256 * data[2]) + data[3]) / 10000 + right_x_offset
        # Right Hand Y Position:
        if data[4] == 0:
            right_py = ((65026 * data[5]) + (256 * data[6]) + data[7]) / 10000 + right_y_offset
        if data[4] == 1:
            right_py = -((65026 * data[5]) + (256 * data[6]) + data[7]) / 10000 + right_y_offset
        # Right Hand Z Position:
        if data[8] == 0:
            right_pz = ((65026 * data[9]) + (256 * data[10]) + data[11]) / 10000 + right_z_offset
        if data[8] == 1:
            right_pz = -((65026 * data[9]) + (256 * data[10]) + data[11]) / 10000 + right_z_offset
        # Right Hand U Rotation:
        if data[12] == 0:
            right_u = ((65026 * data[13]) + (256 * data[14]) + data[15]) / 1000
        if data[12] == 1:
            right_u = -((65026 * data[13]) + (256 * data[14]) + data[15]) / 1000
        # Right Hand V Rotation:
        if data[16] == 0:
            right_v = ((65026 * data[17]) + (256 * data[18]) + data[19]) / 1000
        if data[16] == 1:
            right_v = -((65026 * data[17]) + (256 * data[18]) + data[19]) / 1000
        # Right Hand W Rotation:
        if data[20] == 0:
            right_w = ((65026 * data[21]) + (256 * data[22]) + data[23]) / 1000
        if data[20] == 1:
            right_w = -((65026 * data[21]) + (256 * data[22]) + data[23]) / 1000
        # Right Hand X Velocity:
        if data[24] == 0:
            right_vx = ((65026 * data[25]) + (256 * data[26]) + data[27]) / 10
        if data[24] == 1:
            right_vx = -((65026 * data[25]) + (256 * data[26]) + data[27]) / 10
        # Right Hand Y Velocity:
        if data[28] == 0:
            right_vy = ((65026 * data[29]) + (256 * data[30]) + data[31]) / 10
        if data[28] == 1:
            right_vy = -((65026 * data[29]) + (256 * data[30]) + data[31]) / 10
        # Right Hand Z Velocity:
        if data[32] == 0:
            right_vz = ((65026 * data[33]) + (256 * data[34]) + data[35]) / 10
        if data[32] == 1:
            right_vz = -((65026 * data[33]) + (256 * data[34]) + data[35]) / 10
        # Left Hand X Position:
        if data[36] == 0:
            left_px = ((65026 * data[37]) + (256 * data[38]) + data[39]) / 10000 + left_x_offset
        if data[36] == 1:
            left_px = -((65026 * data[37]) + (256 * data[38]) + data[39]) / 10000 + left_x_offset
        # Left Hand Y Position:
        if data[40] == 0:
            left_py = ((65026 * data[41]) + (256 * data[42]) + data[43]) / 10000 + left_y_offset
        if data[40] == 1:
            left_py = -((65026 * data[41]) + (256 * data[42]) + data[43]) / 10000 + left_y_offset
        # Left Hand Z Position:
        if data[44] == 0:
            left_pz = -((65026 * data[45]) + (256 * data[46]) + data[47]) / 10000 + left_z_offset
        if data[44] == 1:
            left_pz = ((65026 * data[45]) + (256 * data[46]) + data[47]) / 10000 + left_z_offset
        # Left Hand U Rotation:
        if data[48] == 0:
            left_u = ((65026 * data[49]) + (256 * data[50]) + data[51]) / 1000
        if data[48] == 1:
            left_u = -((65026 * data[49]) + (256 * data[50]) + data[51]) / 1000
        # Left Hand V Rotation:
        if data[52] == 0:
            left_v = ((65026 * data[53]) + (256 * data[54]) + data[55]) / 1000
        if data[52] == 1:
            left_v = -((65026 * data[53]) + (256 * data[54]) + data[55]) / 1000
        # Left Hand W Rotation:
        if data[56] == 0:
            left_w = ((65026 * data[57]) + (256 * data[58]) + data[59]) / 1000
        if data[56] == 1:
            left_w = -((65026 * data[57]) + (256 * data[58]) + data[59]) / 1000
        # Left Hand X Velocity:
        if data[60] == 0:
            left_vx = ((65026 * data[61]) + (256 * data[62]) + data[63]) / 10
        if data[60] == 1:
            left_vx = -((65026 * data[61]) + (256 * data[62]) + data[63]) / 10
        # Left Hand Y Velocity:
        if data[64] == 0:
            left_vy = ((65026 * data[65]) + (256 * data[66]) + data[67]) / 10
        if data[64] == 1:
            left_vy = -((65026 * data[65]) + (256 * data[66]) + data[67]) / 10
        # Left Hand Z Velocity:
        if data[68] == 0:
            left_vz = ((65026 * data[69]) + (256 * data[70]) + data[71]) / 10
        if data[68] == 1:
            left_vz = -((65026 * data[69]) + (256 * data[70]) + data[71]) / 10

        # Right Robot Position/Pose Requests
        if Right_Master_Robot_Enabled == True:
            # Actual TCP Pos
            right_pos_actual = rtde_r_r.getActualTCPPose()
            # Actual Joint Pos
            right_q_actual = rtde_r_r.getActualQ()

        # Left Robot Position/Pose Requests
        if Left_Master_Robot_Enabled == True:
            # Actual TCP Pos
            left_pos_actual = rtde_r_l.getActualTCPPose()
            # Actual Joint Pos
            left_q_actual = rtde_r_l.getActualQ()

        # Right Robot Forward Kinematics
        if Right_Kinematics_Enabled == True:
            RightKinematics = ControlCommand(Right_Kinematics_Enabled, link, right_q_actual)

        # Left Robot Forward Kinematics
        if Left_Kinematics_Enabled == True:
            LeftKinematics = ControlCommand(Left_Kinematics_Enabled, link, left_q_actual)

        # Right Hand Quaternion to Axis-Angle Rotations
        if Right_Master_Robot_Enabled == True:
            if Right_EERotations == False:
                right_rot_x = right_pos_actual[3]
                right_rot_y = right_pos_actual[4]
                right_rot_z = right_pos_actual[5]
            if Right_EERotations == True:
                right_quaternion = np.quaternion(1, right_w, right_u, right_v)
                right_axis_angle = quaternion.as_rotation_vector(right_quaternion)
                right_rot_x = right_axis_angle[0]
                right_rot_y = right_axis_angle[2]
                right_rot_z = right_axis_angle[1]


        # Left Hand Quaternion to Axis-Angle Rotations
        if Left_Master_Robot_Enabled == True:
            if Left_EERotations == False:
                left_rot_x = left_pos_actual[3]
                left_rot_y = left_pos_actual[4]
                left_rot_z = left_pos_actual[5]
            if Left_EERotations == True:
                left_quaternion = np.quaternion(1, left_w, left_u, left_v)
                left_axis_angle = quaternion.as_rotation_vector(left_quaternion)
                left_rot_x = left_axis_angle[0]
                left_rot_y = left_axis_angle[2]
                left_rot_z = left_axis_angle[1]


        # Right Hand Pinch and Grab
        right_pinch_strength = ((65026 * data[72]) + (256 * data[73]) + data[74]) / 1000
        right_grab_strength = ((65026 * data[75]) + (256 * data[76]) + data[77]) / 1000

        # Left Hand Pinch and Grab
        left_pinch_strength = ((65026 * data[78]) + (256 * data[79]) + data[80]) / 1000
        left_grab_strength = ((65026 * data[81]) + (256 * data[82]) + data[83]) / 1000

        # Right Fingers to Gripper Position
        gripper_grab_r = round(right_grab_strength * 255)
        gripper_pinch_r = round(right_pinch_strength * 255)

        # Left Fingers to Gripper Position
        gripper_grab_l = round(left_grab_strength * 255)
        gripper_pinch_l = round(left_pinch_strength * 255)

        # Right Robot Absolute Control Calculations
        if Right_Master_Robot_Enabled == True:
            right_r_xy = math.sqrt(pow(right_px, 2) + pow(right_pz, 2))
            right_r_z = math.sqrt(pow(right_r_xy, 2) + pow(right_py, 2))
            right_ee_r_xy = math.sqrt(pow(right_px - right_pos_actual[0], 2) + pow(right_pz - right_pos_actual[2], 2))
            right_ee_r_z = math.sqrt(pow(right_ee_r_xy, 2) + pow(right_py - right_pos_actual[1], 2))
            # Right Hand Angle Quadrant
            if right_px != 0:
                right_r_xy_angle = math.atan(abs(right_pz) / abs(right_px))
            if right_px == 0:
                right_r_xy_angle = math.pi
            if right_r_xy != 0:
                right_r_z_angle = math.atan(abs(right_py) / abs(right_r_xy))

        # Left Robot Absolute Control Calculations
        if Left_Master_Robot_Enabled == True:
            left_r_xy = math.sqrt(pow(left_px, 2) + pow(left_pz, 2))
            left_r_z = math.sqrt(pow(left_r_xy, 2) + pow(left_py, 2))
            left_ee_r_xy = math.sqrt(pow(left_px - left_pos_actual[0], 2) + pow(left_pz - left_pos_actual[2], 2))
            left_ee_r_z = math.sqrt(pow(left_ee_r_xy, 2) + pow(left_py - left_pos_actual[1], 2))
            # Left Hand Angle Quadrant
            if left_px != 0:
                left_r_xy_angle = math.atan(abs(left_pz) / abs(left_px))
            if left_px == 0:
                left_r_xy_angle = math.pi
            if left_r_xy != 0:
                left_r_z_angle = math.atan(abs(left_py) / abs(left_r_xy))

        # Disable Right Robot When Right Hand Tracking is Lost
        if Right_Master_Robot_Enabled == True:
            if ((right_px == right_x_old) & (right_py == right_z_old) & (right_pz == right_z_old)):
                Right_Robot_Enabled = False

        # Disable Left Robot When Left Hand Tracking is Lost
        if Left_Master_Robot_Enabled == True:
            if ((left_px == left_x_old) & (left_py == left_z_old) & (left_pz == left_z_old)):
                Left_Robot_Enabled = False

        # Enable Right Robot When Right Hand Position is Near Right End Effector Position
        if Right_Master_Robot_Enabled == True:
            if ((right_px != right_x_old) & (right_py != right_z_old) & (right_pz != right_z_old)):
                if right_ee_r_z < 0.15:
                    if (gripper_grab_r == 255) & (gripper_grab_l < 255):
                        Right_Robot_Enabled = True

        # Enable Left Robot When Right Hand Position is Near Left End Effector Position
        if Left_Master_Robot_Enabled == True:
            if ((left_px != left_x_old) & (left_py != left_z_old) & (left_pz != left_z_old)):
                if left_ee_r_z < 0.15:
                    if (gripper_grab_l == 255) & (gripper_grab_r < 255):
                        Left_Robot_Enabled = True

        # Hand Off Disable Both Robots
        if (gripper_grab_r == 255) & (gripper_grab_l == 255):
            if Right_Master_Robot_Enabled == True:
                Right_Robot_Enabled = False
            if Left_Master_Robot_Enabled == True:
                Left_Robot_Enabled = False

        # Display Hand Distance From End Effectors
        if (Right_Master_Robot_Enabled == True) & (Left_Master_Robot_Enabled == False):
            print("Right:", right_ee_r_z, Right_Robot_Enabled)
        if (Right_Master_Robot_Enabled == False) & (Left_Master_Robot_Enabled == True):
            print("Left:", left_ee_r_z, Left_Robot_Enabled)
        if (Right_Master_Robot_Enabled == True) & (Left_Master_Robot_Enabled == True):
            print("Right:", right_ee_r_z, Right_Robot_Enabled, "Left:", left_ee_r_z, Left_Robot_Enabled)

        # Right Arm Absolute Control
        if Right_Robot_Enabled == True:
            ControlCommand(rtde_c_r, Control, PID_Follow_Strength, Workspace_Outer, Workspace_Inner, Workspace_Base, gripper_grab_r, right_px, right_py, right_pz, right_rot_x, right_rot_y, right_rot_z, right_r_xy, right_r_z, right_r_xy_angle, right_r_z_angle)

        # Left Arm Absolute Control
        if Left_Robot_Enabled == True:
            ControlCommand(rtde_c_l, Control, PID_Follow_Strength, Workspace_Outer, Workspace_Inner, Workspace_Base, gripper_grab_l, left_px, left_py, left_pz, left_rot_x, left_rot_y, left_rot_z, left_r_xy, left_r_z, left_r_xy_angle, left_r_z_angle)

        # Control Right Gripper Position
        if Right_Master_Robot_Enabled == True:
            if Right_Gripper_Enabled == True:
                # Closed Right Hand Gripper
                if gripper_pinch_r > 0:
                    sgr.send(b'SET POS 255\n')
                # Open Right Hand Gripper
                if gripper_pinch_r == 0:
                    sgr.send(b'SET POS 0\n')

        # Control Right Gripper Position
        if Left_Master_Robot_Enabled == True:
            if Left_Gripper_Enabled == True:
                # Closed Left Hand Gripper
                if gripper_pinch_l > 0:
                    sgl.send(b'SET POS 255\n')
                # Open Left Hand Gripper
                if gripper_pinch_l == 0:
                    sgl.send(b'SET POS 0\n')

        if not data:
            break
        conn.sendall(data)
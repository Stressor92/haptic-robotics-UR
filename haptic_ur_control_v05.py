#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By  : Felix Hillemeier
# Created Date: 30/04/2022
# version ='0.5'
# license = "MIT"
# status = "Research"
# description: Universal Robot (CB control program with ps5-controller and haptic feedback
# valid for: CB3 Software version: from 3.4 forward & e-Series Software versions: All versions
# ---------------------------------------------------------------------------
import rtde_control  # control the UR-robot
import rtde_receive  # receive data from UR-robot control
import tkinter as tk  # GUI
import variables
from time import perf_counter  # returns the float value of time in seconds
from math import exp  # mathematical e function
from pydualsense import pydualsense, TriggerModes  # ps5 controller
from PIL import ImageTk, Image  # picture in gui
from numpy import linalg as la  # Linear algebra
from numpy import rad2deg as rad2deg  # convert radiant in grad

# roboter IP for TCP/IP-Connection
ROBOT_IP: str = variables.ROBOT_IP
# ROBOT_IP: str = "192.168.1.21"   # IP Roboter HWK
# ROBOT_IP: str = "141.55.217.205"  # IP HS Mittweida

SELECTED_ROBOTTYP = variables.SELECTED_ROBOTTYP
SELECTED_SPEED_FACTOR = variables.SELECTED_SPEED_FACTOR

# datarate for the while loop in Hz
DATARATE: int = variables.DATARATE  # CB3 --> max. 125    eSeries --> max. 500

# initialize the time parameter
time_after_loop = perf_counter()

# initialize the robot connection
RTDE_CONTROL = rtde_control.RTDEControlInterface(ROBOT_IP)  # control the UR-robot
RTDE_READ = rtde_receive.RTDEReceiveInterface(ROBOT_IP)  # receive data from UR-robot control

# initialize the controller connection
DUALSENSE = pydualsense()
DUALSENSE.init()


###############  functions  ###################

# function for receiving data from UR-robot control and store it in a list
def receive_roboter_data(rtde_read):
    roboter_data = {}
    roboter_data['actual_q_pose'] = rtde_read.getActualQ()  # Gelenkwinkel des Roboters, Datentyp: 6D Vektor
    roboter_data['actual_TCP_pose'] = rtde_read.getActualTCPPose()  # TCP Position des Roboters, Datentyp: 6D Vektor
    roboter_data['actual_TCP_force'] = rtde_read.getActualTCPForce()  # TCP Kraft und Momente, Datentyp: 6D Vektor
    roboter_data['actual_TCP_speed'] = rtde_read.getActualTCPSpeed()  # TCP Geschwindigkeit, Datentyp: 6D Vektor
    # Returns the current measured TCP speed The speed of the TCP retuned in a pose structure.The first three values
    # are the  cartesian speeds along x, y, z, and the last three define the current rotation axis, rx, ry, rz, and the
    # length | rz, ry, rz | defines the angular velocity in radians / s.
    roboter_data['actual_cartasian_TCP_speed'] = roboter_data['actual_TCP_speed']
    del roboter_data['actual_cartasian_TCP_speed'][2:5]  # Winkelgeschwindigkeit entfernen
    roboter_data['safety_status'] = rtde_read.getSafetyStatusBits()  # Roboterstatus, Datentyp: 6D Vektor
    roboter_data['actual_momentum'] = rtde_read.getActualMomentum()  # Moment des Roboters , Datentyp: double
    return roboter_data


# function for receiving data from ps5 Controller and store it in a list
def receive_controller_data(dualsense):
    controller_data = {}
    controller_data['Circle'] = dualsense.state.circle  # [True, False]
    controller_data['Cross'] = dualsense.state.cross  # [True, False]
    # controller_data['Square'] = dualsense.state.square  # [True, False]
    controller_data['Triangle'] = dualsense.state.triangle  # [True, False]
    # controller_data['L1'] = dualsense.state.L1  # [True, False]
    # controller_data['R1'] = dualsense.state.R1  # [True, False]
    # controller_data['Options'] = dualsense.state.options  # [True, False]
    # controller_data['Share'] = dualsense.state.share  # [True, False]
    controller_data['DpadUp'] = DUALSENSE.state.DpadUp  # [True, False]
    controller_data['DpadDown'] = DUALSENSE.state.DpadDown  # [True, False]
    controller_data['DpadLeft'] = DUALSENSE.state.DpadLeft  # [True, False]
    controller_data['DpadRight'] = DUALSENSE.state.DpadRight  # [True, False]
    controller_data['L_Stick_X'] = dualsense.state.LX  # [-127,128]
    controller_data['L_Stick_Y'] = dualsense.state.LY  # [-127,128]
    controller_data['R_Stick_X'] = dualsense.state.RX  # [-127,128]
    controller_data['R_Stick_Y'] = dualsense.state.RY  # [-127,128]
    controller_data['L2_Stick'] = dualsense.state.L2  # [0,255]
    controller_data['R2_Stick'] = dualsense.state.R2  # [0,255]
    return controller_data


# Gelenke /TCP Geschwindigkeits Inkremente in Abhängigkeit der Datenrate und der R2 Taste
def calculate_roboter_speed(datarate, sony_controller_data, SELECTED_SPEED_FACTOR):
    if SELECTED_SPEED_FACTOR == 'langsam':
        speed_factor = 0.5
    elif SELECTED_SPEED_FACTOR == 'normal':
        speed_factor = 1
    else:
        speed_factor = 2

    roboter_speed = {}
    scalefactor_tcp = 0.00392  # --> max TCP speed = 1m/s
    scalefactor_joint = 0.0041  # --> max joint speed = 60°/s
    roboter_speed['tcp'] = speed_factor * round(scalefactor_tcp * sony_controller_data['R2_Stick'] / datarate, 5)
    roboter_speed['joint'] = speed_factor * round(scalefactor_joint * sony_controller_data['L2_Stick'] / datarate, 5)
    return roboter_speed


# function for receiving force and tourque data from robot Controller and store it in a list
def calculate_force_moment(roboter_data):
    force_moment = {}
    force_moment['Fx'] = abs(roboter_data['actual_TCP_force'][0])
    force_moment['Fy'] = abs(roboter_data['actual_TCP_force'][1])
    force_moment['Fz'] = abs(roboter_data['actual_TCP_force'][2])
    force_moment['force'] = force_moment['Fx'] + force_moment['Fy'] + force_moment['Fz']  # kummulierte Kraft in N
    force_moment['Mx'] = abs(roboter_data['actual_TCP_force'][3])
    force_moment['My'] = abs(roboter_data['actual_TCP_force'][4])
    force_moment['Mz'] = abs(roboter_data['actual_TCP_force'][5])
    force_moment['moment'] = force_moment['Mx'] + force_moment['Mx'] + force_moment['My']  # kummuliertes Moment in Nm
    force_moment['force_moment'] = force_moment['moment'] + force_moment['force']
    return force_moment


# Kraftumrechnung mittels logistscher Funktion als Grundlage des haptischen Feedbacks
def calculate_haptic_feedback(force_moment, roboter_data, SELECTED_ROBOTTYP):
    upper_bound: float = 255  # max. Wert für Vibration & Triggerfeedback ist 255
    lower_bound: float = 0.1  # Minimalwert --> als Int

    logistic_coefficient_force: float = 0.0006  # initialisation for safty

    if SELECTED_ROBOTTYP == 'UR5e':
        logistic_coefficient_force: float = 0.0004  # bestimmt die Form der Kurve -> durch testen ermittelt
    elif SELECTED_ROBOTTYP == 'UR3e':
        logistic_coefficient_force: float = 0.0006  # bestimmt die Form der Kurve -> durch testen ermittelt

    logistic_coefficient_moment: float = 0.002  # bestimmt die Form der Kurve -> durch testen ermittelt
    logistic_coefficient_speed: float = 0.0007  # bestimmt die Form der Kurve -> durch testen ermittelt

    force_feedback = int((upper_bound * lower_bound) / (lower_bound + (upper_bound - lower_bound) * exp(
        -logistic_coefficient_force * force_moment['force'] * upper_bound)))  # [0,255]

    moment_feedback = int((upper_bound * lower_bound) / (lower_bound + (upper_bound - lower_bound) * exp(
        -logistic_coefficient_moment * force_moment['moment'] * upper_bound)))  # [0,255]

    real_speed: float = 1000 * la.norm(
        roboter_data['actual_cartasian_TCP_speed'])  # Geschwindigkeit durch Normierung in mm/s
    # 50-100m/s als maximalwerte je nach Bewegungsart bei Triggertasten 255

    trigger_speed_feedback = int((upper_bound * lower_bound) / (lower_bound + (upper_bound - lower_bound) * exp(
        -logistic_coefficient_speed * real_speed * upper_bound)))  # [0,255]

    trigger_feedback_calc: int = int((trigger_speed_feedback + force_feedback + moment_feedback) / 3)

    # Haptisches Feedback vom Trigger anhand der Kraft + Geschwindigkeit (maximalwert 255)
    trigger_feedback = min(255, trigger_feedback_calc)

    haptic_feedback = {}
    haptic_feedback['speed_feedback'] = int(trigger_speed_feedback)
    haptic_feedback['trigger_feedback'] = int(trigger_feedback)
    haptic_feedback['force_feedback'] = int(force_feedback)
    haptic_feedback['moment_feedback'] = int(moment_feedback)
    return haptic_feedback


# Feedback des Controllers
def controller_feedback(dualsense, haptic_feedback):
    # Vibration anhand der Kraft festlegen --> Rechts Momente, links Kraft
    dualsense.setLeftMotor(haptic_feedback['force_feedback'])
    dualsense.setRightMotor(haptic_feedback['moment_feedback'])

    # Haptisches Feedback an die Trigger übertragen
    dualsense.triggerR.setMode(TriggerModes.Rigid)
    dualsense.triggerR.setForce(1, haptic_feedback['trigger_feedback'])  # [0,255]
    dualsense.triggerL.setMode(TriggerModes.Rigid)
    dualsense.triggerL.setForce(1, haptic_feedback['trigger_feedback'])  # [0,255]

    # Licht als Visuelles Feedback --> Hohe Kraft/Momenteinwirkung = rot / geringe Kraft/Momenteinwirkung = grün
    dualsense.light.setColorI(max(0, haptic_feedback['force_feedback'] - 5),
                              max(0, 250 - haptic_feedback['force_feedback']), 0)


def roboter_control(rtde_control, sony_controller_data, roboter_data, roboter_speed):
    # Initiale Werte für die servoJ Funktion - zum Steuern des Roboters
    velocity: float = 0.5  # Geschwindigkeit
    acceleration: float = 0.5  # Beschleunigung
    dt: float = 1.0 / 500  # 2ms - Zeit, in der der Befehl den Roboter steuert
    lookahead_t: float = 0.1  # glättet die Trajektorie mit der definierten Vorlaufzeit  [0.03,0.2]
    gain: float = 300  # Proportionalverstärkung für Zielposition [100,2000]

    # aktuelle Position des Roboters
    joint_q = roboter_data['actual_q_pose']
    actual_tcp_pose = roboter_data['actual_TCP_pose']

    # Bewegung des Roboters, wenn die entsprechende Taste gedrück wird
    if sony_controller_data['L_Stick_X'] > 50:
        joint_q[0] += roboter_speed['joint']
        actual_tcp_pose[0] += roboter_speed['tcp']

    if sony_controller_data['L_Stick_X'] < -50:
        joint_q[0] -= roboter_speed['joint']
        actual_tcp_pose[0] -= roboter_speed['tcp']

    if sony_controller_data['L_Stick_Y'] > 50:
        joint_q[1] += roboter_speed['joint']
        actual_tcp_pose[1] -= roboter_speed['tcp']

    if sony_controller_data['L_Stick_Y'] < -50:
        joint_q[1] -= roboter_speed['joint']
        actual_tcp_pose[1] += roboter_speed['tcp']

    if sony_controller_data['R_Stick_X'] > 50:
        joint_q[2] += roboter_speed['joint']
        actual_tcp_pose[3] += roboter_speed['tcp']

    if sony_controller_data['R_Stick_X'] < -50:
        joint_q[2] -= roboter_speed['joint']
        actual_tcp_pose[3] -= roboter_speed['tcp']

    if sony_controller_data['R_Stick_Y'] > 50:
        joint_q[3] += roboter_speed['joint']
        actual_tcp_pose[3] += 3 * roboter_speed['tcp']

    if sony_controller_data['R_Stick_Y'] < -50:
        joint_q[3] -= roboter_speed['joint']
        actual_tcp_pose[3] -= 3 * roboter_speed['tcp']

    if sony_controller_data['DpadUp'] == 1:
        joint_q[4] -= roboter_speed['joint']
        actual_tcp_pose[4] += 3 * roboter_speed['tcp']

    if sony_controller_data['DpadDown'] == 1:
        joint_q[4] += roboter_speed['joint']
        actual_tcp_pose[4] -= 3 * roboter_speed['tcp']

    if sony_controller_data['DpadLeft'] == 1:
        joint_q[5] += roboter_speed['joint']
        actual_tcp_pose[5] += 3 * roboter_speed['tcp']

    if sony_controller_data['DpadRight'] == 1:
        joint_q[5] -= roboter_speed['joint']
        actual_tcp_pose[5] -= 3 * roboter_speed['tcp']

    # moves the robot in joint-movement with the servoJ function
    if roboter_speed['joint'] > 0:  # if L2 is pressed
        rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_t, gain)

    # moves the robot in tcp-movement, with calculating the inverse kinematics and use the servoJ function
    elif roboter_speed['tcp'] > 0:  # if R2 is pressed
        tcp_position_to_q = rtde_control.getInverseKinematics(actual_tcp_pose)  # calculate the inverse kinematics
        rtde_control.servoJ(tcp_position_to_q, velocity, acceleration, dt, lookahead_t, gain)

    # NOT-Halt = x
    if controller_data['Cross'] == 1:
        rtde_control.triggerProtectiveStop()

    # Freedrive when circle is pressed
    if controller_data['Circle'] == 1:
        rtde_control.freedriveMode()
    if controller_data['Circle'] == 0:
        rtde_control.endFreedriveMode()

    # zero the force sensor with triangle
    if controller_data['Triangle'] == 1:
        rtde_control.zeroFtSensor()
    return


# Update den Wert in R2_Stick_value
def update_gui_data():
    global controller_data
    global roboter_data
    global force_moment
    global haptic_feedback
    global roboter_speed
    global real_datarate
    global SELECTED_ROBOTTYP
    global SELECTED_SPEED_FACTOR

    runden4 = 4
    runden2 = 2

    color = '#229954'
    color_emergency = '#ff0000'

    datarate_value.config(text=round(real_datarate, 0))  # show real datarate

    SELECTED_ROBOTTYP = selected_option.get()  # get the variable for selected robotertype
    SELECTED_SPEED_FACTOR = selected_speed.get()  # get the variable for selected roboterspeed

    # red text if the robot is close to max rotation
    if -6. < roboter_data['actual_q_pose'][0] < 6.:
        robot_position_value1.config(text=round(rad2deg(roboter_data['actual_q_pose'][0]), runden2), fg=color)
    else:
        robot_position_value1.config(text=round(rad2deg(roboter_data['actual_q_pose'][0]), runden2), fg=color_emergency)

    if -6.< roboter_data['actual_q_pose'][1] < 6.:
        robot_position_value2.config(text=round(rad2deg(roboter_data['actual_q_pose'][1]), runden2), fg=color)
    else:
        robot_position_value2.config(text=round(rad2deg(roboter_data['actual_q_pose'][1]), runden2), fg=color_emergency)

    if -6. < roboter_data['actual_q_pose'][2] < 6.:
        robot_position_value3.config(text=round(rad2deg(roboter_data['actual_q_pose'][2]), runden2), fg=color)
    else:
        robot_position_value3.config(text=round(rad2deg(roboter_data['actual_q_pose'][2]), runden2), fg=color_emergency)

    if -6.< roboter_data['actual_q_pose'][3] < 6.:
        robot_position_value4.config(text=round(rad2deg(roboter_data['actual_q_pose'][3]), runden2), fg=color)
    else:
        robot_position_value4.config(text=round(rad2deg(roboter_data['actual_q_pose'][3]), runden2), fg=color_emergency)

    if -6. < roboter_data['actual_q_pose'][4] < 6.:
        robot_position_value5.config(text=round(rad2deg(roboter_data['actual_q_pose'][4]), runden2), fg=color)
    else:
        robot_position_value5.config(text=round(rad2deg(roboter_data['actual_q_pose'][4]), runden2), fg=color_emergency)

    if -6. < roboter_data['actual_q_pose'][5] < 6.:
        robot_position_value6.config(text=round(rad2deg(roboter_data['actual_q_pose'][5]), runden2), fg=color)
    else:
        robot_position_value6.config(text=round(rad2deg(roboter_data['actual_q_pose'][5]), runden2), fg=color_emergency)

    # roboter_data['actual_q_pose'] max min wert [-6.28318530718,6.28318530718]

    robot_position_value7.config(text=round(1000 * roboter_data['actual_TCP_pose'][0], runden2), fg=color)
    robot_position_value8.config(text=round(1000 * roboter_data['actual_TCP_pose'][1], runden2), fg=color)
    robot_position_value9.config(text=round(1000 * roboter_data['actual_TCP_pose'][2], runden2), fg=color)
    robot_position_value10.config(text=round(rad2deg(roboter_data['actual_TCP_pose'][3]), runden2), fg=color)
    robot_position_value11.config(text=round(rad2deg(roboter_data['actual_TCP_pose'][4]), runden2), fg=color)
    robot_position_value12.config(text=round(rad2deg(roboter_data['actual_TCP_pose'][5]), runden2), fg=color)

    kraft_moment_value1.config(text=round(roboter_data['actual_TCP_force'][0], runden4), fg=color)
    kraft_moment_value2.config(text=round(roboter_data['actual_TCP_force'][1], runden4), fg=color)
    kraft_moment_value3.config(text=round(roboter_data['actual_TCP_force'][2], runden4), fg=color)
    kraft_moment_value4.config(text=round(roboter_data['actual_TCP_force'][3], runden4), fg=color)
    kraft_moment_value5.config(text=round(roboter_data['actual_TCP_force'][4], runden4), fg=color)
    kraft_moment_value6.config(text=round(roboter_data['actual_TCP_force'][5], runden4), fg=color)
    # [-50,50]
    kraft_moment_value7.config(text=round(force_moment['force'], 2), fg=color)
    kraft_moment_value8.config(text=round(force_moment['moment'], 2), fg=color)

    # Norm des Geschwindigkeitsvektors
    speed_normiert = la.norm(roboter_data['actual_cartasian_TCP_speed'])
    kraft_moment_value9.config(text=round(speed_normiert, runden4), fg=color)
    kraft_moment_value10.config(text=round(roboter_data['actual_momentum'], runden4), fg=color)
    # Sicherheitsstatus vom Robotersystem
    SafetyStatus = 'running'
    if roboter_data['safety_status'] == 1:
        SafetyStatus = "normal mode"
    elif roboter_data['safety_status'] == 2:
        SafetyStatus = "reduced mode"
    elif roboter_data['safety_status'] == 3:
        SafetyStatus = "protective stopped"
    elif roboter_data['safety_status'] == 4:
        SafetyStatus = 'recovery mode'
    elif roboter_data['safety_status'] == 5:
        SafetyStatus = 'system emergency stopped'
    elif roboter_data['safety_status'] == 6:
        SafetyStatus = 'robot emergency stopped'
    elif roboter_data['safety_status'] == 7:
        SafetyStatus = 'emergency stopped'
    elif roboter_data['safety_status'] == 8:
        SafetyStatus = 'violation'
    elif roboter_data['safety_status'] == 9:
        SafetyStatus = 'fault'
    elif roboter_data['safety_status'] == 10:
        SafetyStatus = 'stopped due to safety'
    kraft_moment_value11.config(text=SafetyStatus, fg=color)

    if controller_data['L_Stick_X'] > 50:
        controller_values1.config(text='1', fg=color)
    if controller_data['L_Stick_X'] < -50:
        controller_values1.config(text='-1', fg=color)
    if -25 < controller_data['L_Stick_X'] < 50:
        controller_values1.config(text='0', fg=color)

    if controller_data['L_Stick_Y'] > 50:
        controller_values2.config(text='1', fg=color)
    if controller_data['L_Stick_Y'] < -50:
        controller_values2.config(text='-1', fg=color)
    if -25 < controller_data['L_Stick_Y'] < 50:
        controller_values2.config(text='0', fg=color)

    if controller_data['R_Stick_X'] > 50:
        controller_values3.config(text='1', fg=color)
    if controller_data['R_Stick_X'] < -50:
        controller_values3.config(text='-1', fg=color)
    if -25 < controller_data['R_Stick_X'] < 50:
        controller_values3.config(text='0', fg=color)

    if controller_data['R_Stick_Y'] > 50:
        controller_values4.config(text='1', fg=color)
    if controller_data['R_Stick_Y'] < -50:
        controller_values4.config(text='-1', fg=color)
    if -25 < controller_data['R_Stick_Y'] < 50:
        controller_values4.config(text='0', fg=color)

    controller_values5.config(text=controller_data['DpadUp'], fg=color)
    controller_values6.config(text=controller_data['DpadDown'], fg=color)
    controller_values7.config(text=controller_data['L2_Stick'], fg=color)
    controller_values8.config(text=controller_data['R2_Stick'], fg=color)
    controller_values9.config(text=round(roboter_speed['joint'], runden4), fg=color)
    controller_values10.config(text=round(roboter_speed['tcp'], runden4), fg=color)
    controller_values11.config(text=haptic_feedback['trigger_feedback'], fg=color)
    controller_values12.config(text=haptic_feedback['force_feedback'], fg=color)
    controller_values13.config(text=haptic_feedback['moment_feedback'], fg=color)

    # GUI anhand der Fenstergröße anpassen --> je Höher, desto tiefer liegen die Messwerte
    height = window.winfo_height()
    window.grid_rowconfigure(0, minsize=height - 400)

    # GUI anhand der Fenstergröße anpassen --> je Breiter, desto weiter rechts liegen die Messwerte
    width = window.winfo_width()
    window.grid_columnconfigure(0, minsize=(width - 950) / 2)
    return


def gui_disconnect():
    global RTDE_CONTROL
    global DUALSENSE
    DUALSENSE.close()  # close the connection of controller
    RTDE_CONTROL.disconnect()  # close the connection of robot
    window.destroy()  # close the window
    return


########################  GUI ###########################

window = tk.Tk()  # creat a window
window.title('haptic feedback UR Control')  # titel
window['background'] = '#FFFFFF'  # backgroundcolor
window.iconbitmap('ur16e.ico')  # Icon
window.geometry('1000x800')  # resolution
window.minsize(1000, 750)  # minimal size of window
window.state('zoomed')  # start as maxed

# picture of the control
controller_image = ImageTk.PhotoImage(Image.open('Controller.png'))
image_label = tk.Label(image=controller_image)
image_label.place(relx=0.5, rely=0.25, anchor='center')

# size of column and rows (and empty ones for looks)
window.grid_columnconfigure(0, minsize=50)
window.grid_columnconfigure(2, minsize=100)  # Breite der Values Zeile
window.grid_columnconfigure(3, minsize=100)
window.grid_columnconfigure(5, minsize=100)  # Breite der Values Zeile
window.grid_columnconfigure(6, minsize=100)
window.grid_columnconfigure(8, minsize=100)  # Breite der Values Zeile
window.grid_rowconfigure(0, minsize=400)
window.grid_rowconfigure(15, minsize=50)

# fonts
font_headline = ('Courier', 14)
font_headline2 = ('Courier', 18)
font_values = ('Courier', 10)
font_data = ('Courier', 10)

# Drop-down Robot selection
roboter_options = [
    'UR3e',
    'UR5e',
]
selected_option = tk.StringVar()
selected_option.set(roboter_options[0])

dropdown_option = tk.OptionMenu(window, selected_option, *roboter_options)
dropdown_option.place(relx=0.9, rely=0.025)

# Drop-down speed selection
roboter_speed_factor = [
    'langsam',
    'normal',
    'schnell',
]
selected_speed = tk.StringVar()
selected_speed.set(roboter_speed_factor[1])

dropdown_speed = tk.OptionMenu(window, selected_speed, *roboter_speed_factor)
dropdown_speed.place(relx=0.9, rely=0.065)

# Lable für die IP Adresse une Datenrate
ip_adress_label = tk.Label(window, text="IP-Adresse:", font=font_headline2)
ip_adress_label.place(relx=0.4, rely=0.42, anchor='center')
ip_adress_label = tk.Label(window, text=ROBOT_IP, font=font_headline2, fg='#8f90a8')
ip_adress_label.place(relx=0.6, rely=0.42, anchor='center')
datarate_label = tk.Label(window, text="Datenübertragungsrate:", font=font_headline2)
datarate_label.place(relx=0.4, rely=0.455, anchor='center')
datarate_label = tk.Label(window, text=DATARATE, font=font_headline2, fg='#8f90a8')
datarate_label.place(relx=0.6, rely=0.455, anchor='center')
datarate_label = tk.Label(window, text="real:", font=font_values)
datarate_label.place(relx=0.66, rely=0.455, anchor='center')
datarate_value = tk.Label(window, text=0.00, font=font_values, fg='#8f90a8')
datarate_value.place(relx=0.7, rely=0.455, anchor='center')

# Überschriften der 3 Wertkategorien
robot_position_label = tk.Label(window, text="Roboter Position", font=font_headline)
robot_position_label.grid(row=1, column=1, columnspan=2)
kraft_moment_label = tk.Label(window, text="Kräfte und Drehmomente", font=font_headline)
kraft_moment_label.grid(row=1, column=4, columnspan=2)
controller_label = tk.Label(window, text="Controller Daten", font=font_headline)
controller_label.grid(row=1, column=7, columnspan=2)

# Datenbeschriftung
robot_position_data = tk.Label(window, text="Joint1 [°]", font=font_data)
robot_position_data.grid(row=2, column=1)
robot_position_data = tk.Label(window, text="Joint2 [°]", font=font_data)
robot_position_data.grid(row=3, column=1)
robot_position_data = tk.Label(window, text="Joint3 [°]", font=font_data)
robot_position_data.grid(row=4, column=1)
robot_position_data = tk.Label(window, text="Joint4 [°]", font=font_data)
robot_position_data.grid(row=5, column=1)
robot_position_data = tk.Label(window, text="Joint5 [°]", font=font_data)
robot_position_data.grid(row=6, column=1)
robot_position_data = tk.Label(window, text="Joint6 [°]", font=font_data)
robot_position_data.grid(row=7, column=1)
robot_position_data = tk.Label(window, text="TCP X [mm]", font=font_data)
robot_position_data.grid(row=8, column=1)
robot_position_data = tk.Label(window, text="TCP Y [mm]", font=font_data)
robot_position_data.grid(row=9, column=1)
robot_position_data = tk.Label(window, text="TCP Z [mm]", font=font_data)
robot_position_data.grid(row=10, column=1)
robot_position_data = tk.Label(window, text="TCP RX [°]", font=font_data)
robot_position_data.grid(row=11, column=1)
robot_position_data = tk.Label(window, text="TCP RY [°]", font=font_data)
robot_position_data.grid(row=12, column=1)
robot_position_data = tk.Label(window, text="TCP RZ [°]", font=font_data)
robot_position_data.grid(row=13, column=1)

kraft_moment_data = tk.Label(window, text="Kraft X [N]", font=font_data)
kraft_moment_data.grid(row=2, column=4)
kraft_moment_data = tk.Label(window, text="Kraft y [N]", font=font_data)
kraft_moment_data.grid(row=3, column=4)
kraft_moment_data = tk.Label(window, text="Kraft Z [N]", font=font_data)
kraft_moment_data.grid(row=4, column=4)
kraft_moment_data = tk.Label(window, text="Drehmoment X [Nm]", font=font_data)
kraft_moment_data.grid(row=5, column=4)
kraft_moment_data = tk.Label(window, text="Drehmoment Y [Nm]", font=font_data)
kraft_moment_data.grid(row=6, column=4)
kraft_moment_data = tk.Label(window, text="Drehmoment Z [Nm]", font=font_data)
kraft_moment_data.grid(row=7, column=4)
kraft_moment_data = tk.Label(window, text="Summe Kraft [N]", font=font_data)
kraft_moment_data.grid(row=8, column=4)
kraft_moment_data = tk.Label(window, text="Summe Moment [Nm]", font=font_data)
kraft_moment_data.grid(row=9, column=4)
kraft_moment_data = tk.Label(window, text="TCP-Geschwindigkeit [m/s]", font=font_data)
kraft_moment_data.grid(row=10, column=4)
kraft_moment_data = tk.Label(window, text="Drehmoment [Nm]", font=font_data)
kraft_moment_data.grid(row=11, column=4)
kraft_moment_data = tk.Label(window, text="SafetyStatus", font=font_data)
kraft_moment_data.grid(row=12, column=4)

controller_data = tk.Label(window, text="L Stick X", font=font_data)
controller_data.grid(row=2, column=7)
controller_data = tk.Label(window, text="L Stick Y", font=font_data)
controller_data.grid(row=3, column=7)
controller_data = tk.Label(window, text="R Stick X", font=font_data)
controller_data.grid(row=4, column=7)
controller_data = tk.Label(window, text="R Stick Y", font=font_data)
controller_data.grid(row=5, column=7)
controller_data = tk.Label(window, text="Steuerkreuz ⇅", font=font_data)
controller_data.grid(row=6, column=7)
controller_data = tk.Label(window, text="Steuerkreuz ⇆", font=font_data)
controller_data.grid(row=7, column=7)
controller_data = tk.Label(window, text="L2", font=font_data)
controller_data.grid(row=8, column=7)
controller_data = tk.Label(window, text="R2", font=font_data)
controller_data.grid(row=9, column=7)
controller_data = tk.Label(window, text="Joint-Inkrement", font=font_data)
controller_data.grid(row=10, column=7)
controller_data = tk.Label(window, text="TCP-Inkrement", font=font_data)
controller_data.grid(row=11, column=7)
controller_data = tk.Label(window, text="Trigger Feedback", font=font_data)
controller_data.grid(row=12, column=7)
controller_data = tk.Label(window, text="rechter Vibrator", font=font_data)
controller_data.grid(row=13, column=7)
controller_data = tk.Label(window, text="linker Vibrator", font=font_data)
controller_data.grid(row=14, column=7)

# Datenwerte
robot_position_value1 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value1.grid(row=2, column=2)
robot_position_value2 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value2.grid(row=3, column=2)
robot_position_value3 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value3.grid(row=4, column=2)
robot_position_value4 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value4.grid(row=5, column=2)
robot_position_value5 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value5.grid(row=6, column=2)
robot_position_value6 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value6.grid(row=7, column=2)
robot_position_value7 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value7.grid(row=8, column=2)
robot_position_value8 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value8.grid(row=9, column=2)
robot_position_value9 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value9.grid(row=10, column=2)
robot_position_value10 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value10.grid(row=11, column=2)
robot_position_value11 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value11.grid(row=12, column=2)
robot_position_value12 = tk.Label(window, text="0.0000", font=font_values)
robot_position_value12.grid(row=13, column=2)

kraft_moment_value1 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value1.grid(row=2, column=5)
kraft_moment_value2 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value2.grid(row=3, column=5)
kraft_moment_value3 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value3.grid(row=4, column=5)
kraft_moment_value4 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value4.grid(row=5, column=5)
kraft_moment_value5 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value5.grid(row=6, column=5)
kraft_moment_value6 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value6.grid(row=7, column=5)
kraft_moment_value7 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value7.grid(row=8, column=5)
kraft_moment_value8 = tk.Label(window, text="0.00", font=font_values)
kraft_moment_value8.grid(row=9, column=5)
kraft_moment_value9 = tk.Label(window, text="0.0", font=font_values)
kraft_moment_value9.grid(row=10, column=5)
kraft_moment_value10 = tk.Label(window, text="0.0", font=font_values)
kraft_moment_value10.grid(row=11, column=5)
kraft_moment_value11 = tk.Label(window, text=" ", font=font_values)
kraft_moment_value11.grid(row=12, column=5)

controller_values1 = tk.Label(window, text="0", font=font_values)
controller_values1.grid(row=2, column=8)
controller_values2 = tk.Label(window, text="0", font=font_values)
controller_values2.grid(row=3, column=8)
controller_values3 = tk.Label(window, text="0", font=font_values)
controller_values3.grid(row=4, column=8)
controller_values4 = tk.Label(window, text="0", font=font_values)
controller_values4.grid(row=5, column=8)
controller_values5 = tk.Label(window, text="0", font=font_values)
controller_values5.grid(row=6, column=8)
controller_values6 = tk.Label(window, text="0", font=font_values)
controller_values6.grid(row=7, column=8)
controller_values7 = tk.Label(window, text="0", font=font_values)
controller_values7.grid(row=8, column=8)
controller_values8 = tk.Label(window, text="0", font=font_values)
controller_values8.grid(row=9, column=8)
controller_values9 = tk.Label(window, text="0", font=font_values)
controller_values9.grid(row=10, column=8)
controller_values10 = tk.Label(window, text="0", font=font_values)
controller_values10.grid(row=11, column=8)
controller_values11 = tk.Label(window, text="0", font=font_values)
controller_values11.grid(row=12, column=8)
controller_values12 = tk.Label(window, text="0", font=font_values)
controller_values12.grid(row=13, column=8)
controller_values13 = tk.Label(window, text="0", font=font_values)
controller_values13.grid(row=14, column=8)

# Disconnect Button
disconnect_button = tk.Button(window,
                              text="Disconnect",
                              padx=45, pady=5,
                              command=gui_disconnect,
                              bg="#CEF6F5",
                              fg="black")
disconnect_button.grid(row=16, column=4, columnspan=3)

##############    Hauptprogamm   ######################

while True:

    time_before_loop = perf_counter()  # time varible before the loop

    # if loot which get executed in a specific frequency/datarate
    if time_before_loop - time_after_loop >= 1. / DATARATE:
        real_datarate = 1. / (time_before_loop - time_after_loop)  # real frequency

        # main programm/functions
        roboter_data = receive_roboter_data(RTDE_READ)
        controller_data = receive_controller_data(DUALSENSE)
        force_moment = calculate_force_moment(roboter_data)
        roboter_speed = calculate_roboter_speed(DATARATE, controller_data, SELECTED_SPEED_FACTOR)
        haptic_feedback = calculate_haptic_feedback(force_moment, roboter_data, SELECTED_ROBOTTYP)
        controller_feedback(DUALSENSE, haptic_feedback)
        roboter_control(RTDE_CONTROL, controller_data, roboter_data, roboter_speed)

        # GUI update
        window.update()
        window.after_idle(update_gui_data)  # update the gui with the function

        time_after_loop = perf_counter()  # time varible after the loop

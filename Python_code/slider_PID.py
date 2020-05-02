from tkinter import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import socket
from struct import unpack
from struct import pack
import sys
from inputs import devices
import os
import pprint
import pygame

style.use('ggplot')


#file = open(r"/home/pi/Poseitron-Code/Data/PID.txt", "w")
file = open(r"/home/matteofdc/Documents/Poseitron_Data/Data/PID.txt", "w")

UDP_IP = "192.168.1.111"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


master = Tk()
master.geometry("1300x1300")
master.title("Live PID tuning")
w1 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kp-left')
w1.set(0.04)
w1.pack()

w2 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Ki-left')
w2.set(0.7)
w2.pack()

w3 = Scale(master, from_=0, to=0.0005, length=600, digits=5,
           resolution=0.000001, orient=HORIZONTAL, label='Kd-left')
w3.set(0.00004)
w3.pack()

w4 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kp-right')
w4.set(0.04)
w4.pack()

w5 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Ki-right')
w5.set(0.7)
w5.pack()

w6 = Scale(master, from_=0, to=0.0005, length=600, digits=5,
           resolution=0.000001, orient=HORIZONTAL, label='Kd-right')
w6.set(0.00004)
w6.pack()

w7 = Scale(master, from_=0, to=2, length=600, digits=4,
           resolution=0.001, orient=HORIZONTAL, label='Left speed correction factor')
w7.set(1)
w7.pack()

w8 = Scale(master, from_=0, to=2, length=600, digits=4,
           resolution=0.001, orient=HORIZONTAL, label='Right speed correction factor')
w8.set(1)
w8.pack()


Ki_left = w1.get()
Kp_left = w2.get()
Kd_left = w3.get()
Ki_right = w4.get()
Kp_right = w5.get()
Kd_right = w6.get()
Left_correction = w7.get()
Right_correction = w8.get()
ps4_left_y = 0
ps4_left_x = 0
ps4_right_y = 0
ps4_right_x = 0


class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""

        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self, ready):
        """Listen for events to happen"""

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        if ready:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value, 2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.

                os.system('clear')
                pprint.pprint(self.button_data)
                pprint.pprint(self.axis_data)

                if 0 in self.axis_data:
                    global ps4_left_x
                    ps4_left_x = self.axis_data[0]
                if 1 in self.axis_data:
                    global ps4_left_y
                    ps4_left_y = -self.axis_data[1]
                if 3 in self.axis_data:
                    global ps4_right_x
                    ps4_right_x = self.axis_data[3]
                if 4 in self.axis_data:
                    global ps4_right_y
                    ps4_right_y = -self.axis_data[4]
                pprint.pprint(self.hat_data)


'''if __name__ == "__main__":'''
ps4 = PS4Controller()
ps4.init()


def print_values_in_file():
    global Ki_left
    Ki_left = w1.get()
    global Kp_left
    Kp_left = w2.get()
    global Kd_left
    Kd_left = w3.get()
    global Ki_right
    Ki_right = w4.get()
    global Kp_right
    Kp_right = w5.get()
    global Kd_right
    Kd_right = w6.get()


def print_correction():
    global Left_correction
    Left_correction = w7.get()
    global Right_correction
    Right_correction = w8.get()


def resetPID():
    w1.set(0.04)
    w2.set(0.7)
    w3.set(0.00004)
    w4.set(0.04)
    w5.set(0.7)
    w6.set(0.00004)
    print_values_in_file()


def zeroPID():
    w1.set(0)
    w2.set(0)
    w3.set(0)
    w4.set(0)
    w5.set(0)
    w6.set(0)
    print_values_in_file()


def speed(left_x, left_y, right_x, right_y):
    zeSpeed = 150
    lin_speed = left_y * zeSpeed
    left_speed = lin_speed * (right_x+1)/2
    right_speed = lin_speed * (-right_x+1)/2
    if lin_speed == 0:
        left_speed = zeSpeed * right_x/2
        right_speed = zeSpeed*-right_x/2
    return [left_speed, right_speed]


Button(master, text='Update PID', command=print_values_in_file).pack()
Button(master, text='Reset PID', command=resetPID).pack()
Button(master, text='ZERO PID', command=zeroPID).pack()
Button(master, text='Update correction factors',
       command=print_correction).pack()
# Button(master,)


fig, axs = plt.subplots(2, 2, figsize=(30, 30))

axs[1, 0].set_xlim([-1, 1])
axs[1, 0].set_ylim([-1, 1])

axs[1, 1].set_xlim([-1, 1])
axs[1, 1].set_ylim([-1, 1])


class L(list):
    def append(self, item):
        list.append(self, item)
        if len(self) > 100:
            del self[0]


Vr = L()
VrRef = L()
Vl = L()
VlRef = L()
Time = L()


def animate(i):
    # f = open(r"/home/pi/Poseitron-Code/Data/logFileSpeed.txt", "r").read()
    ps4.listen(True)
    speeds = speed(ps4_left_x, ps4_left_y, ps4_right_x, ps4_right_y)
    PID = [float(Ki_left), float(Kp_left), float(Kd_left),
           float(Ki_right), float(Kp_right), float(Kd_right), float(Left_correction), float(Right_correction), float(speeds[0]), float(speeds[1])]
    data = pack('ffffffffff', *PID)
   # print("MSG PID = {}".format(data))
    sock.sendto(data, (UDP_IP, UDP_PORT))
    msg = sock.recv(40)
    data = unpack('<5d', msg)
   # print("MSG = {}".format(data))
   # print("PID = {}".format(PID))
    Vr.append(float(data[0]))
    VrRef.append(float(data[1]))
    Vl.append(float(data[2]))
    VlRef.append(float(data[3]))
    Time.append(float(data[4]))
    axs[0, 1].clear()
    axs[0, 1].plot(Time, Vr)
    axs[0, 1].plot(Time, VrRef)
    axs[0, 1].set_title("Right Motor Speed VS reference")
    axs[0, 0].clear()
    axs[0, 0].plot(Time, Vl)
    axs[0, 0].plot(Time, VlRef)
    axs[0, 0].set_title("Left Motor Speed VS reference")
    axs[1, 0].clear()
    axs[1, 0].plot([1, 1, -1, -1, ps4_left_x], [1, -1, 1, -
                                                1, ps4_left_y], marker='o', color='r', ls='')
    axs[1, 0].plot([1, 1, -1, -1, ps4_right_x], [1, -1, 1, -
                                                 1, ps4_right_y], marker='o', color='g', ls='')
    #axs[1, 1].clear()


chart_type = FigureCanvasTkAgg(fig, master)
chart_type.get_tk_widget().pack()

ani = animation.FuncAnimation(fig, animate, interval=1)


master.mainloop()

from tkinter import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import socket
from struct import unpack
from struct import pack

style.use('ggplot')

#file = open(r"/home/pi/Poseitron-Code/Data/PID.txt", "w")
file = open(r"/home/matteofdc/Documents/Poseitron_Data/Data/PID.txt", "w")

UDP_IP = "192.168.1.111"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


master = Tk()
master.geometry("1200x1200")
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

Ki_left = w1.get()
Kp_left = w2.get()
Kd_left = w3.get()
Ki_right = w4.get()
Kp_right = w5.get()
Kd_right = w6.get()


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


Button(master, text='Update PID', command=print_values_in_file).pack()


fig, axs = plt.subplots(2, 1, figsize=(20, 20))


class L(list):
    def append(self, item):
        list.append(self, item)
        if len(self) > 500:
            del self[0]


Vr = L()
VrRef = L()
Vl = L()
VlRef = L()
Time = L()


def animate(i):
    # f = open(r"/home/pi/Poseitron-Code/Data/logFileSpeed.txt", "r").read()
    PID = [float(Ki_left),float(Kp_left),float(Kd_left),float(Ki_right),float(Kp_right),float(Kd_right)]
    data= pack('>6f',*PID)
    sock.sendto(data, (UDP_IP, UDP_PORT))
    msg = sock.recv(40)
    data = unpack('<5d', msg)
    print("MSG = {}".format(data))
    Vr.append(float(data[0]))
    VrRef.append(float(data[1]))
    Vl.append(float(data[2]))
    VlRef.append(float(data[3]))
    Time.append(float(data[4]))
    axs[0].clear()
    axs[0].plot(Time, Vr)
    axs[0].plot(Time, VrRef)
    axs[0].set_title("Right Motor Speed VS reference")
    axs[1].clear()
    axs[1].plot(Time, Vl)
    axs[1].plot(Time, VlRef)
    axs[1].set_title("Left Motor Speed VS reference")


chart_type = FigureCanvasTkAgg(fig, master)
chart_type.get_tk_widget().pack()

ani = animation.FuncAnimation(fig, animate, interval=1)


master.mainloop()

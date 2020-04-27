from Tkinter import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import os

style.use('ggplot')

file = open(r"/home/pi/Poseitron-Code/Data/PID.txt", "w")
#file = open(r"/home/matteofdc/Documents/Poseitron_Data/Data/PID.txt", "w")


def print_values_in_file():
    file.seek(0)
    file.write(str(w1.get())+' ')
    file.write(str(w2.get())+' ')
    file.write(str(w3.get())+' ')
    file.write(str(w4.get())+' ')
    file.write(str(w5.get())+' ')
    file.write(str(w6.get())+'\n')


master = Tk()
master.geometry("1200x1200")
master.title("Live PID tuning")
w1 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kp-left')
w1.set(0)
w1.pack()

w2 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Ki-left')
w2.set(0)
w2.pack()

w3 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kd-left')
w3.set(0)
w3.pack()

w4 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kp-right')
w4.set(0)
w4.pack()

w5 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Ki-right')
w5.set(0)
w5.pack()

w6 = Scale(master, from_=0, to=1, length=600, digits=4,
           resolution=0.0001, orient=HORIZONTAL, label='Kd-right')
w6.set(0)
w6.pack()


Button(master, text='Update PID', command=print_values_in_file).pack()


fig, axs = plt.subplots(2, 1, figsize=(20, 20))

Vr = []
VrRef = []
Vl = []
VlRef = []
Time = []


def animate(i):
    f = open('/home/pi/Poseitron-Code/Data/logFileSpeed.txt', 'r')
    # next(f)

    for line in f:
        line = line.strip()
        vr, vrref, vl, vlref, time = line.split()
        Vr.append(float(vr))
        VrRef.append(float(vrref))
        Vl.append(float(vl))
        VlRef.append(float(vlref))
        Time.append(float(time))
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

ani = animation.FuncAnimation(fig, animate, interval=10)


master.mainloop()

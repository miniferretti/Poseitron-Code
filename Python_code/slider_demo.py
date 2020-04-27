from Tkinter import *

def print_values_in_file():
    print (w1.get(), w2.get())

master = Tk()
w1 = Scale(master, from_=0, to=1,length=600,digits=4,resolution=0.0001,orient=HORIZONTAL,label='Kp-left')
w1.set(0)
w1.pack()
w2 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001,orient=HORIZONTAL, label='ki-left')
w2.set(0)
w2.pack()
w3 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001,orient=HORIZONTAL, label='kd-left')
w3.set(0)
w3.pack()

w4 = Scale(master, from_=0, to=1,length=600,digits=4,resolution=0.0001,orient=HORIZONTAL,label='Kp-right')
w4.set(0)
w4.pack()
w5 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001,orient=HORIZONTAL, label='ki-right')
w5.set(0)
w5.pack()
w6 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001,orient=HORIZONTAL, label='kd-right')
w6.set(0)
w6.pack()



Button(master, text='Show', command=print_values_in_file).pack()

mainloop()
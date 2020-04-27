from Tkinter import *

def print_values_in_file():
    print (w1.get(), w2.get())

master = Tk()
w1 = Scale(master, from_=0, to=1,length=600,digits=4,resolution=0.0001,orient=HORIZONTAL,label='Kp')
w1.set(0)
w1.pack()
w2 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001,orient=HORIZONTAL, label='ki')
w2.set(0)
w2.pack()
Button(master, text='Show', command=print_values_in_file).pack()

mainloop()
from Tkinter import *

def show_values():
    print (w1.get(), w2.get())

master = Tk()
w1 = Scale(master, from_=0, to=1,length=600,digits=4,resolution=0.0001)
w1.set(0)
w1.pack()
w2 = Scale(master, from_=0, to=1, length=600,digits=4,resolution=0.0001)
w2.set(0)
w2.pack()
Button(master, text='Show', command=show_values).pack()

mainloop()
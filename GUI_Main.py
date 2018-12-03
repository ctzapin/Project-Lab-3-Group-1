import serial
from tkinter import *
import tkinter
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
#====== VARIABLES ===========================================================

COM = "COM13" #COM port
BAUD = 9600   #Baudrate
forward = 'w' #move forward
reverse = 's' #move reverse
left    = 'a' #turn left
right   = 'd' #turn right
stop    = ' ' #stop movement
scan    = 'S' #do one scan
deliver = 'D' #do one delivery
mode_A  = '1' #automatic mode
mode_M  = '2' #manual mode
mode_P  = '3' #path follow mode
mode = 0
comand = 0
xDisplace = 0
yDisplace = 0
moveDur = 0


#====== SERIAL COM =======================================================================================

ser = serial.Serial(COM, BAUD)
ser.close()
ser.open()

root = Tk() #========================================== Begining

#GUI definitions
def send_stop():
    global moveDur
    ser.write(b' ')
    print("sent stop")
    #moveDur = ord(ser.read())

def send_forward():
    ser.write(b'w')
    print("sent forward")

def send_reverse():
    ser.write(b's')
    print("sent reverse")

def send_left():
    ser.write(b'a')
    print("sent left")

def send_right():
    ser.write(b'd')
    print("sent right")

def send_scan():
    global comand
    comand = "S"
    ser.write(b'S')
    print("sent scan")

def send_deliver():
    ser.write(b'D')
    print("sent deliver")

def send_mode_A():
    global mode
    mode = "automatic"
    ser.write(b'1')
    print("sent auto mode")

def send_mode_M():
    global mode
    mode = "manual"
    ser.write(b'2')
    print("sent manual mode")

def send_mode_P():
    global mode
    mode = "path folower"
    ser.write(b'3')
    print("sent path mode")
    #enterCord()
    enterDir()

def Quit_App():
    ser.close()
    root.quit()
    root.destroy()

def enterCord():
    global xDisplace, yDisplace
    def saveCord():
        xDisplace = XE.get()
        yDisplace = YE.get()
        print(xDisplace, yDisplace)

    newwin = Toplevel(root)
    title = Label(newwin, text="Path Follow")
    XL = Label(newwin, text="X-Cord")
    YL = Label(newwin, text="Y-Cord")
    XE = Entry(newwin, bd = 5)
    YE = Entry(newwin, bd = 5)
    title.grid(row=0, column=0)
    XL.grid(row=1, column=1)
    YL.grid(row=2, column=1)
    XE.grid(row=1, column=2)
    YE.grid(row=2, column=2)
    save_button = tkinter.Button(master=newwin, text="save", command=saveCord)
    save_button.grid(row=5, column=2)

def enterDir():
    global pathDirections

    def addDir():
        pathDirections.append([ str(Dir.get()), str(Dur.get()) ])
        LB.insert(END, pathDirections[-1])
        print(pathDirections)

    def removeDir():
        num = LB.curselection()
        LB.delete(num)
        del pathDirections[num[0]]
        print(pathDirections)

    def sendDir():
        for i in range(len(pathDirections)):
            ser.write(pathDirections[i][0].encode())
            ser.write(pathDirections[i][1].encode())
            print("sent", pathDirections[i][0].encode(), pathDirections[i][1].encode())

    pathDirections = []
    newwin = Toplevel(root)
    title = Label(newwin, text="Path Follow")
    DirL =  Label(newwin, text="direction")
    DurL =  Label(newwin, text="duration")
    Dir = Entry(newwin, bd=5)
    Dur  = Entry(newwin, bd=5)

    LB = Listbox(newwin)
    LB.grid(row=0, column=0, rowspan=20)

    DirL.grid(row=0, column=1)
    DurL.grid(row=1, column=1)
    Dir.grid( row=0, column=2)
    Dur.grid( row=1, column=2)

    addButton = tkinter.Button(master=newwin, text="add", command=addDir)
    addButton.grid(row=5, column=2)

    removeButton = tkinter.Button(master=newwin, text="remove", command=removeDir)
    removeButton.grid(row=6, column=2)

    sendButton = tkinter.Button(master=newwin, text="send", command=sendDir)
    sendButton.grid(row=7, column=2)




def resetPlot():
    global xdata, ydata, adata, mode, dist, ang, rad, x, y, xDisplace, yDisplace

    xdata = []
    ydata = []
    adata = []
    xDisplace = 0
    yDisplace = 0

def connect():
    ser.open()
    print("connected to", COM, "at", BAUD, "baudrate")

def disconnect():
    ser.close()
    print("disconnected")



# ======== CONTROL =============================================================
stop_button = tkinter.Button(master=root, text="stop", command=send_stop)
stop_button.grid(row=4, column=2)

forward_button = tkinter.Button(master=root, text="forward", command=send_forward)
forward_button.grid(row=1, column=2)

reverse_button = tkinter.Button(master=root, text="reverse", command=send_reverse)
reverse_button.grid(row=3, column=2)

left_button = tkinter.Button(master=root, text="left", command=send_left)
left_button.grid(row=2, column=0)

right_button = tkinter.Button(master=root, text="right", command=send_right)
right_button.grid(row=2, column=3)

scan_button = tkinter.Button(master=root, text="scan", command=send_scan)
scan_button.grid(row=2, column=6)

deliver_button = tkinter.Button(master=root, text="deliver", command=send_deliver)
deliver_button.grid(row=3, column=6)

#=========== modes ==============================================

automatic_mode = tkinter.Button(master=root, text="automatic", command=send_mode_A)
automatic_mode.grid(row=2, column=8)

manual_mode = tkinter.Button(master=root, text="manual", command=send_mode_M)
manual_mode.grid(row=3, column=8)

path_mode = tkinter.Button(master=root, text="path follow", command=send_mode_P)
path_mode.grid(row=4, column=8)

#========= OTHER ===============================

Quit_button = tkinter.Button(master=root, text="Quit", command=Quit_App)
Quit_button.grid(row=5, column=2)

reset_button = tkinter.Button(master=root, text="Reset", command=resetPlot)
reset_button.grid(row=5, column=5)

connectB = tkinter.Button(master=root, text="connect", command=connect)
connectB.grid(row=5, column=10)

disconnectB = tkinter.Button(master=root, text="disconnect", command=disconnect)
disconnectB.grid(row=5, column=11)



#================== GUI 2 ==================================================================
def GUI2():
    newwin = Toplevel(root)

    def connect():
        ser.open()
        print("connected to", COM, "at", BAUD, "baudrate")

    connectB = tkinter.Button(master=newwin, text="connect", command=connect)

    def disconnect():
        ser.close()
        print("disconnected")

    disconnectB = tkinter.Button(master=newwin, text="disconnect", command=disconnect)

    def EnterPathFollow():
        ser.write(b'a')
        print("sent a")

    EnterPathFollowB = tkinter.Button(master=newwin, text="Enter Path Follow", command=EnterPathFollow)

    def ExitPathFollow():
        ser.write(b'b')
        print("sent b")

    ExitPathFollowB = tkinter.Button(master=newwin, text="Exit Path Follow", command=ExitPathFollow)

    def PrepareDirections():
        ser.write(b'c')
        print("sent c")
        enterDir()

    PrepareDirectionsB = tkinter.Button(master=newwin, text="Prepare Directions", command=PrepareDirections)

    def FinishDirections():
        ser.write(b'd')
        print("sent d")

    FinishDirectionsB = tkinter.Button(master=newwin, text="Finish Directions", command=FinishDirections)

    def EnterScanningMode():
        ser.write(b'A')
        print("sent A")

    EnterScanningModeB = tkinter.Button(master=newwin, text="Enter ScanningMode", command=EnterScanningMode)

    def ExitScanningMode():
        ser.write(b'b')
        print("sent B")

    ExitScanningModeB = tkinter.Button(master=newwin, text="Exit ScanningMode", command=ExitScanningMode)

    connectB.grid(row=0, column=0)
    disconnectB.grid(row=0, column=1)

    EnterPathFollowB.grid(row=1, column=0)
    ExitPathFollowB.grid(row=1, column=1)

    PrepareDirectionsB.grid(row=2, column=0)

    EnterScanningModeB.grid(row=3, column=0)
    ExitScanningModeB.grid(row=3, column=1)

def OpenGUI2():
    ser.close()
    GUI2()
OpenGUI2B = tkinter.Button(master=root, text="GUI2", command=OpenGUI2)
OpenGUI2B.grid(row = 10, column = 20)


#===========================================
count = 0
xdata = []
ydata = []
adata = []

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
sc = ax.scatter(xdata, ydata)
plt.xlim(-200,200)
plt.ylim(-200,200)

def print_data(count, dist, ang, x, y, xdata, ydata, adata):
    print("count: ", count)
    print("distance: ", dist, "angle: ", ang)
    print("x-cord: ", x, ",", "y-cord: ",  y)
    print(xdata)
    print(ydata)
    print(adata)
    print("")


def Plot():
    global count, xdata, ydata, adata, mode, dist, ang, rad, x, y, xDisplace, yDisplace
    # ser.open()
    dist = ord(ser.read())
    ang = ord(ser.read())
    # ser.close()

    rad = np.deg2rad(ang)
    x = np.cos(rad) * dist
    y = np.sin(rad) * dist

    x = int(round(x))
    y = int(round(y))
    a = int(round(ang))

    xdata = np.append(xdata, x+xDisplace)
    ydata = np.append(ydata, y+yDisplace)
    adata = np.append(adata, a)

    print(xDisplace, yDisplace)

    count = count + 1
    print_data(count, dist, ang, x, y, xdata, ydata, adata)

    sc.set_offsets(np.c_[xdata, ydata])
    ax.scatter(xDisplace, yDisplace, c="red")
    #ax.scatter(xdata, ydata)
    root.after(1)

def animate(i):
    global count, xdata, ydata, adata, mode, \
           comand, xDisplace, yDisplace, dist, ang, rad, x, y, moveDur

    if (mode == "automatic"):
        counter = 0
        while (counter != 180):
            Plot()
            counter = ang
        #moveDur = ord(ser.read())
        # xDisplace = xDisplace
        # yDisplace = yDisplace + 50


    elif (mode == "manual"):
        if (comand == "S"):
            Plot()
            while (ang != 180):
                Plot()
            comand = 0

    elif (mode == "path folower"):
        x=0



plotcanvas = FigureCanvasTkAgg(fig, root)
plotcanvas.get_tk_widget().grid(row=0,column=0, columnspan=15)
ani = animation.FuncAnimation(fig, animate, interval=10, blit=False)

root.mainloop() #=========================================================== END




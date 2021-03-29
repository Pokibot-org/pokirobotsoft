import matplotlib.pyplot as plt
import matplotlib.animation as anim
import serial
import time
import threading

mutex = threading.Lock()

def update(i, ax, list_x, list_y):
    mutex.acquire()
    ax.clear()
    ax.scatter(list_x, list_y, c= 'red')
    ax.set_xlim([-1000,1000])
    ax.set_ylim([-1000,1000])
    list_x.clear()
    list_y.clear()
    mutex.release()

def read_points_from_serial(list_x, list_y):
    
    with serial.Serial("COM5", baudrate=524288) as ser:
        while True:
            try:
                line = ser.read_until(b'\n').decode().replace("\n",'').replace("\r",'')
                if not line.startswith("<") or not line.endswith(">"):
                    continue
                x, y = [int(a) for a in line.replace(">", "").replace("<","").split(":")]
                mutex.acquire()
                list_x.append(x) 
                list_y.append(y)
                mutex.release()
            except:
                pass

if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    list_x = []
    list_y = []
    t = threading.Thread(target=read_points_from_serial, args=(list_x, list_y))
    t.daemon = True
    t.start()
    a = anim.FuncAnimation(fig, update, fargs=(ax, list_x, list_y), repeat=True, interval=200)
    plt.show()
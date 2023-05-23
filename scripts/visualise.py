import serial
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import struct

PORT_NAME = 'COM1'
DMAX = 4000
IMIN = 0
IMAX = 50

N_BINS = 720

def update_line(num, serial, line):
    serial.flushInput()
    l = serial.readline()
    l = serial.readline()
    m = l.decode("utf-8").split(',')[:-1]
    if not len(m) == N_BINS:
        print(len(m))
        return line,
    scan = [(i * 360 / N_BINS, float(x)) for i, x in enumerate(m)]

    offsets = np.array([(np.radians(meas[0]), meas[1]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([0] * N_BINS)
    line.set_array(intens)
    return line,

def run():
    ser = serial.Serial(PORT_NAME, 115200, timeout=5)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    ani = animation.FuncAnimation(fig, update_line,
        fargs=(ser, line), interval=10)
    plt.show()
    ser.close()

if __name__ == '__main__':
    run()

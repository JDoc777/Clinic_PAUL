import serial
import struct
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys

# ---------- SERIAL CONFIG ----------
PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.05)
buf = bytearray()

# ---------- QT APP ----------
app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(title="PAUL – LiDAR Live Scan")
win.resize(800, 800)
win.show()

plot = win.addPlot()
plot.setAspectLocked(True)
plot.setXRange(-4000, 4000)
plot.setYRange(-4000, 4000)
plot.setLabel('bottom', 'X (mm)')
plot.setLabel('left', 'Y (mm)')
plot.showGrid(x=True, y=True)

scatter = pg.ScatterPlotItem(size=4, pen=None, brush=pg.mkBrush(0, 255, 0, 150))
plot.addItem(scatter)

# ---------- UPDATE LOOP ----------
def update():
    global buf

    buf += ser.read(2048)
    points = []

    while True:
        start = buf.find(b'\xAA\x55')
        if start < 0 or len(buf) < start + 12:
            break

        length = buf[start + 2]
        if len(buf) < start + length:
            break

        frame = buf[start:start + length]
        buf = buf[start + length:]

        angle_deg = 0
        for i in range(8, min(len(frame) - 1, 120), 2):
            d = struct.unpack_from('<H', frame, i)[0]
            if 50 < d < 8000:
                theta = np.deg2rad(angle_deg)
                x = d * np.cos(theta)
                y = d * np.sin(theta)
                points.append((x, y))
                angle_deg += 2

    if points:
        scatter.setData(pos=np.array(points))


# ---------- TIMER ----------
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)   # ~33 FPS

sys.exit(app.exec())

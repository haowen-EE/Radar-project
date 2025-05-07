# Updated readData_AWR1843.py with debug prints

import os
import glob
import time
import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.opengl import GLViewWidget, GLScatterPlotItem

# ─── 配置文件自动发现 ─────────────────────────────
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
cfgs = glob.glob(os.path.join(desktop, '*.cfg'))
if len(cfgs) != 1:
    print("请确保桌面上恰好有一个 .cfg 文件，当前找到：", cfgs)
    raise SystemExit(1)
configFileName = cfgs[0]
print("使用配置文件：", configFileName)

# ─── 全局变量 ───────────────────────────────────
CLIport = None
Dataport = None
byteBuffer = np.zeros(2**15, dtype=np.uint8)
byteBufferLength = 0

# ─── 串口配置 & 下发配置文件 ─────────────────────
def serialConfig(cfgName):
    global CLIport, Dataport
    # TODO: 根据实际设备管理器查看的端口号修改
    CLIport  = serial.Serial('COM4', 115200)
    Dataport = serial.Serial('COM3', 921600)

    with open(cfgName, 'r') as f:
        for line in f:
            cmd = line.strip()
            print("SENDCFG:", cmd)
            CLIport.write((cmd + '\n').encode())
            time.sleep(0.01)

    print("SEND: sensorStart")
    CLIport.write(b'sensorStart\n')
    time.sleep(0.01)
    return CLIport, Dataport

# ─── 解析配置文件 ────────────────────────────────
def parseConfigFile(cfgName):
    configParameters = {}
    lines = [line.strip() for line in open(cfgName)]
    for line in lines:
        parts = line.split()
        if parts[0] == 'profileCfg':
            startFreq = int(float(parts[2]))
            idleTime = int(parts[3])
            rampEndTime = float(parts[5])
            freqSlopeConst = float(parts[8])
            numAdcSamples = int(parts[10])
            numAdcSamplesRoundTo2 = 1
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 *= 2
            digOutSampleRate = int(parts[11])
        elif parts[0] == 'frameCfg':
            chirpStartIdx = int(parts[1])
            chirpEndIdx = int(parts[2])
            numLoops = int(parts[3])
            numFrames = int(parts[4])
            framePeriodicity = float(parts[5])

    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = int(numChirpsPerFrame / 3)  # numTxAnt=3
    configParameters["numRangeBins"]   = numAdcSamplesRoundTo2
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamplesRoundTo2)
    configParameters["dopplerResolutionMps"] = 3e8 / (
        2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 *
        configParameters["numDopplerBins"] * 3)
    return configParameters

# ─── 读取并解析 TLV 数据 ─────────────────────────
def readAndParseData18xx(port, configParameters):
    global byteBuffer, byteBufferLength

    # Debug: show how many bytes are waiting
    waiting = port.in_waiting
    print(f"DEBUG: {waiting} bytes waiting on Dataport")

    MMWDEMO_UART_MSG_DETECTED_POINTS        = 1
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
    magicWord = [2,1,4,3,6,5,8,7]
    maxBufferSize = 2**15

    dataOK = False
    frameNumber = 0
    detObj = {}
    rdMat = None

    # 读取串口
    n = port.in_waiting or 1
    buff = port.read(n)
    vec = np.frombuffer(buff, dtype=np.uint8)
    cnt = len(vec)

    if byteBufferLength + cnt < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength+cnt] = vec
        byteBufferLength += cnt

    if byteBufferLength > len(magicWord):
        idxs = np.where(byteBuffer == magicWord[0])[0]
        starts = [i for i in idxs if np.all(byteBuffer[i:i+8] == magicWord)]
        if starts:
            offset = starts[0]
            byteBuffer[:byteBufferLength-offset] = byteBuffer[offset:byteBufferLength]
            byteBufferLength -= offset
            totalLen = int(np.dot(byteBuffer[12:16], [1,2**8,2**16,2**24]))
            if byteBufferLength >= totalLen:
                dataOK = True
                print(f"DEBUG: got full packet, length={totalLen}")

                i = 0
                i += 8+4+4+4  # skip header
                frameNumber = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24]))
                i += 4
                numObj = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                numTLVs = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4

                for _ in range(numTLVs):
                    tlv_type = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                    tlv_len  = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4

                    if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                        print(f"DEBUG: TLV type 1 — detected points: {numObj}")
                        xs = np.zeros(numObj, dtype=np.float32)
                        ys = np.zeros(numObj, dtype=np.float32)
                        zs = np.zeros(numObj, dtype=np.float32)
                        for o in range(numObj):
                            xs[o] = byteBuffer[i:i+4].view('float32'); i+=4
                            ys[o] = byteBuffer[i:i+4].view('float32'); i+=4
                            zs[o] = byteBuffer[i:i+4].view('float32'); i+=4
                        detObj = {'numObj': numObj, 'x': xs, 'y': ys, 'z': zs}

                    elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                        print("DEBUG: TLV type 5 — rangeDoppler matrix received")
                        rb = configParameters["numRangeBins"]
                        db = configParameters["numDopplerBins"]
                        numBytes = 2*rb*db
                        payload = byteBuffer[i:i+numBytes]; i += numBytes
                        mat = payload.view(np.int16).reshape((db, rb), order='F')
                        half = db//2
                        rdMat = np.vstack((mat[half:], mat[:half]))

                    else:
                        i += tlv_len

                byteBuffer[:byteBufferLength-totalLen] = byteBuffer[totalLen:byteBufferLength]
                byteBufferLength -= totalLen

    return dataOK, frameNumber, detObj, rdMat

# ─── 定时更新函数 ─────────────────────────────────
def update():
    global scatter3d, rd_img
    dataOk, frm, detObj, rdMat = readAndParseData18xx(Dataport, configParameters)
    print(f"DEBUG: update() returned dataOk={dataOk}")
    if dataOk:
        if detObj.get('numObj',0)>0:
            print("DEBUG: updating 3D scatter with", detObj['numObj'], "points")
            pts = np.vstack((detObj['x'], detObj['y'], detObj['z'])).T
            scatter3d.setData(pos=pts, size=0.05, color=(1,0,0,1))
        if rdMat is not None:
            print("DEBUG: updating RD plot, shape=", rdMat.shape)
            rd_img.setImage(rdMat, autoLevels=True)

# ─── 主入口 ─────────────────────────────────────
if __name__ == '__main__':
    CLIport, Dataport      = serialConfig(configFileName)
    configParameters       = parseConfigFile(configFileName)

    app = QtWidgets.QApplication([])

    # 3D Scatter Plot
    view3d = GLViewWidget(); view3d.setWindowTitle('3D Scatter'); view3d.opts['distance']=3
    scatter3d = GLScatterPlotItem(); view3d.addItem(scatter3d); view3d.show()

    # Range-Doppler Plot
    rd_win = pg.GraphicsLayoutWidget(title='Range-Doppler')
    rd_plot= rd_win.addPlot(); rd_plot.setLabel('left','Doppler'); rd_plot.setLabel('bottom','Range')
    rd_img = pg.ImageItem(); rd_plot.addItem(rd_img); rd_win.show()

    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(50)
    app.exec_()

    CLIport.write(b'sensorStop\n'); CLIport.close(); Dataport.close()

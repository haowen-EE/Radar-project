import os
import sys
import serial
import serial.tools.list_ports
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.opengl as gl

# 自动加载桌面唯一 .cfg 文件
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
cfgs = [f for f in os.listdir(desktop) if f.lower().endswith('.cfg')]
if len(cfgs) != 1:
    print('请确保桌面上恰好有一个 .cfg 文件，目前:', cfgs)
    sys.exit(1)
configFileName = os.path.join(desktop, cfgs[0])
print('使用配置文件:', configFileName)

# 全局变量
CLIport = None
Dataport = None
byteBuffer = np.zeros(2**15, dtype='uint8')
byteBufferLength = 0
configParams = None

# ------------------------------------------------------------------
# 串口配置并下发
```python
def serialConfig(cfgName):
    global CLIport, Dataport
    # 手动指定端口
    CLIport = serial.Serial('COM3', 115200)
    Dataport = serial.Serial('COM4', 921600)
    with open(cfgName, 'r') as f:
        for line in f:
            cmd = line.strip()
            CLIport.write((cmd + '\n').encode())
            print('SENDCFG:', cmd)
            time.sleep(0.01)
    CLIport.write(b'sensorStart\n')
    print('SEND: sensorStart')
    time.sleep(0.01)
    return CLIport, Dataport
```
# ------------------------------------------------------------------
# 配置解析
def parseConfigFile(cfgName):
    lines = [l.strip() for l in open(cfgName)]
    numRxAnt, numTxAnt = 4, 3
    for ln in lines:
        parts = ln.split()
        if parts[0] == 'profileCfg':
            startFreq = float(parts[2]); idleTime = float(parts[3]); rampEndTime = float(parts[5])
            freqSlope = float(parts[8]); numAdc = int(parts[10])
            adcsRound = 1
            while adcsRound < numAdc:
                adcsRound <<= 1
            digRate = int(parts[11])
        elif parts[0] == 'frameCfg':
            chirpStart = int(parts[1]); chirpEnd = int(parts[2]); numLoops = int(parts[3])
    numChirps = (chirpEnd - chirpStart + 1) * numLoops
    return {
        'numDopplerBins': numChirps // numTxAnt,
        'numRangeBins': adcsRound,
        'rangeIdx2m': (3e8 * digRate * 1e3) / (2 * freqSlope * 1e12 * adcsRound),
        'dopplerRes': 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * (numChirps // numTxAnt) * numTxAnt)
    }

# ------------------------------------------------------------------
# 数据解析 (点 + Range-Doppler)
def readAndParseData18xx(Dataport, cfg):
    MMWDET, MMWRD = 1, 5
    maxSize, magic = 2**15, [2,1,4,3,6,5,8,7]
    dataOK, detObj, rdMat = False, {}, None
    sz = Dataport.in_waiting
    if sz:
        raw = Dataport.read(sz)
        buf = np.frombuffer(raw, dtype='uint8')
        if byteBufferLength + len(buf) < maxSize:
            byteBuffer[byteBufferLength:byteBufferLength+len(buf)] = buf
            byteBufferLength += len(buf)
    if byteBufferLength > len(magic):
        idxs = np.where(byteBuffer == magic[0])[0]
        starts = [i for i in idxs if np.all(byteBuffer[i:i+len(magic)]==magic)]
        if starts:
            s0 = starts[0]
            byteBuffer[:byteBufferLength-s0] = byteBuffer[s0:byteBufferLength]
            byteBufferLength -= s0
            totalLen = int(np.matmul(byteBuffer[12:16],[1,2**8,2**16,2**24]))
            if byteBufferLength >= totalLen:
                dataOK = True
                i = 0; i += 20
                frameNum = int(np.matmul(byteBuffer[i:i+4],[1,2**8,2**16,2**24])); i += 4
                numDet = int(np.matmul(byteBuffer[i:i+4],[1,2**8,2**16,2**24])); i += 4
                numTLV= int(np.matmul(byteBuffer[i:i+4],[1,2**8,2**16,2**24])); i += 4
                for _ in range(numTLV):
                    tlvType = int(np.matmul(byteBuffer[i:i+4],[1,2**8,2**16,2**24])); i += 4
                    tlvLen  = int(np.matmul(byteBuffer[i:i+4],[1,2**8,2**16,2**24])); i += 4
                    if tlvType == MMWDET:
                        x=np.zeros(numDet); y=np.zeros(numDet); z=np.zeros(numDet)
                        for k in range(numDet):
                            x[k] = byteBuffer[i:i+4].view('float32'); i += 4
                            y[k] = byteBuffer[i:i+4].view('float32'); i += 4
                            z[k] = byteBuffer[i:i+4].view('float32'); i += 4
                        detObj = {'numObj':numDet,'x':x,'y':y,'z':z}
                    elif tlvType == MMWRD:
                        nr, nd = cfg['numRangeBins'], cfg['numDopplerBins']
                        cnt = 2*nr*nd
                        pld = byteBuffer[i:i+cnt]; i += cnt
                        rd = pld.view('int16').reshape((nd,nr),order='F')
                        rd = np.vstack((rd[nd//2:], rd[:nd//2]))
                        rdMat = rd
                    else:
                        i += tlvLen
                byteBuffer[:byteBufferLength-totalLen] = byteBuffer[totalLen:byteBufferLength]
                byteBufferLength -= totalLen
    return dataOK, detObj, rdMat

# 更新绘制
def update():
    ok, dobj, rmat = readAndParseData18xx(Dataport, configParams)
    if ok:
        if 'x' in dobj:
            pts = np.vstack((dobj['x'], dobj['y'], dobj['z'])).T
            scatter3d.setData(pos=pts, size=5, color=(1,1,1,1))
        if rmat is not None:
            img.setImage(rmat, autoLevels=False)

# 主程序入口
if __name__ == '__main__':
    CLIport, Dataport = serialConfig(configFileName)
    configParams = parseConfigFile(configFileName)
    app = QtWidgets.QApplication([])
    # 3D 散点图
    w3d = gl.GLViewWidget(); w3d.setWindowTitle('3D Scatter Plot'); w3d.setCameraPosition(distance=2)
    scatter3d = gl.GLScatterPlotItem(); w3d.addItem(scatter3d); w3d.show()
    # Range-Doppler 热图
    win2 = pg.GraphicsLayoutWidget(title='Range-Doppler Plot')
    p2 = win2.addPlot(); img = pg.ImageItem(); p2.addItem(img); win2.show()
    # 定时更新
    tmr = QtCore.QTimer(); tmr.timeout.connect(update); tmr.start(50)
    app.exec_()
    # 退出清理
    CLIport.write(b'sensorStop\n'); CLIport.close(); Dataport.close()

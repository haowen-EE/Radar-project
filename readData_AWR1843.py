import os
import sys
import serial
import serial.tools.list_ports
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# 调试：打印当前目录及可用 .cfg 文件
print("当前工作目录：", os.getcwd())
print("可用配置文件：", [f for f in os.listdir(os.getcwd()) if f.endswith('.cfg')])

# 自动加载桌面唯一 .cfg 文件
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
cfg_list = [f for f in os.listdir(desktop) if f.lower().endswith('.cfg')]
if len(cfg_list) != 1:
    print("请确保桌面上恰好有一个 .cfg 文件，当前找到：", cfg_list)
    sys.exit(1)
configFileName = os.path.join(desktop, cfg_list[0])
print("使用配置文件：", configFileName)

# 全局变量
CLIport = None
Dataport = None
byteBuffer = np.zeros(2**15, dtype='uint8')
byteBufferLength = 0
configParameters = None

# ------------------------------------------------------------------
# 自动检测 CP2105 串口并发送配置、启动命令
def serialConfig(cfgName):
    global CLIport, Dataport
    # 自动检测 CP2105 虚拟串口
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"Port: {p.device}, Desc: {p.description}")
    cp_ports = [p for p in ports if 'CP2105' in p.description]
    if len(cp_ports) < 2:
        raise RuntimeError(f"未找到两个 CP2105 端口，发现: {[p.device for p in cp_ports]}")
    # 区分 Standard/Enhanced
    std = next((p.device for p in cp_ports if 'Standard' in p.description), cp_ports[0].device)
    enh = next((p.device for p in cp_ports if 'Enhanced' in p.description), cp_ports[1].device)
    print(f"Using CLIport={std}, Dataport={enh}")
    # 打开 CLIport，如果失败则尝试交换端口
    try:
        CLIport = serial.Serial(std, 115200)
    except serial.SerialException as e:
        print(f"无法打开CLI端口 {std}: {e}，尝试交换端口")
        std, enh = enh, std
        print(f"交换后 Using CLIport={std}, Dataport={enh}")
        CLIport = serial.Serial(std, 115200)
    # 打开 Dataport
    Dataport = serial.Serial(enh, 921600)
    # 下发配置
    with open(cfgName, 'r') as f:
        for line in f:
            cmd = line.strip()
            CLIport.write((cmd + '
').encode())
            print("SENDCFG:", cmd)
            time.sleep(0.01)
    # 启动雷达
    CLIport.write(b'sensorStart
')
    print("SEND: sensorStart")
    time.sleep(0.01)
    return CLIport, Dataport

# ------------------------------------------------------------------
# 解析配置文件参数
def parseConfigFile(cfgName):
    lines = [ln.strip() for ln in open(cfgName)]
    numRxAnt, numTxAnt = 4, 3
    for ln in lines:
        parts = ln.split()
        if parts[0] == 'profileCfg':
            startFreq = float(parts[2]); idleTime = float(parts[3]); rampEndTime = float(parts[5])
            freqSlope = float(parts[8]); numAdc = int(parts[10])
            adcsRound = 1
            while adcsRound < numAdc:
                adcsRound <<= 1
            digOutRate = int(parts[11])
        elif parts[0] == 'frameCfg':
            chirpStart = int(parts[1]); chirpEnd = int(parts[2]); numLoops = int(parts[3])
    numChirps = (chirpEnd - chirpStart + 1) * numLoops
    params = {
        'numDopplerBins': numChirps // numTxAnt,
        'numRangeBins': adcsRound,
        'rangeIdx2m': (3e8 * digOutRate * 1e3) / (2 * freqSlope * 1e12 * adcsRound),
        'dopplerRes': 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * (numChirps // numTxAnt) * numTxAnt)
    }
    return params

# ------------------------------------------------------------------
# 解析串口数据包，返回检测点
def readAndParseData18xx(Dataport, cfgParams):
    global byteBuffer, byteBufferLength
    MDEMO_UART_MSG_DETECTED = 1
    maxSize = 2**15
    magicWord = [2,1,4,3,6,5,8,7]
    dataOK = False; frameNumber = 0; detObj = {}
    size = Dataport.in_waiting
    if size:
        raw = Dataport.read(size)
        buf = np.frombuffer(raw, dtype='uint8')
        if byteBufferLength + len(buf) < maxSize:
            byteBuffer[byteBufferLength: byteBufferLength+len(buf)] = buf
            byteBufferLength += len(buf)
    if byteBufferLength > len(magicWord):
        idxs = np.where(byteBuffer == magicWord[0])[0]
        st = [i for i in idxs if np.all(byteBuffer[i:i+len(magicWord)] == magicWord)]
        if st:
            s0 = st[0]
            byteBuffer[: byteBufferLength-s0] = byteBuffer[s0:byteBufferLength]
            byteBufferLength -= s0
            pktLen = int(np.matmul(byteBuffer[12:16], [1,2**8,2**16,2**24]))
            if byteBufferLength >= pktLen:
                dataOK = True
                i = 0; i += 8+4+4+4
                frameNumber = int(np.matmul(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                numDet = int(np.matmul(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                numTLV = int(np.matmul(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                for _ in range(numTLV):
                    tlvT = int(np.matmul(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                    tlvL = int(np.matmul(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
                    if tlvT == MDEMO_UART_MSG_DETECTED:
                        x = np.zeros(numDet, dtype=np.float32)
                        y = np.zeros(numDet, dtype=np.float32)
                        for k in range(numDet):
                            x[k] = byteBuffer[i:i+4].view('float32'); i+=4
                            y[k] = byteBuffer[i:i+4].view('float32'); i+=4
                            i += 8
                        detObj = {'numObj':numDet, 'x':x, 'y':y}
                    else:
                        i += tlvL
                byteBuffer[: byteBufferLength-pktLen] = byteBuffer[pktLen:byteBufferLength]
                byteBufferLength -= pktLen
    return dataOK, frameNumber, detObj

# 定时更新绘图
def update():
    ok, fn, dobj = readAndParseData18xx(Dataport, configParameters)
    if ok and 'x' in dobj and len(dobj['x']):
        s.setData(-dobj['x'], dobj['y'])
    return ok

# 主程序
if __name__ == '__main__':
    CLIport, Dataport = serialConfig(configFileName)
    configParameters = parseConfigFile(configFileName)
    app = QtWidgets.QApplication([])
    pg.setConfigOption('background', 'w')
    win = pg.GraphicsLayoutWidget(title="2D scatter plot")
    p = win.addPlot()
    p.setXRange(-0.5,0.5); p.setYRange(0,1.5)
    p.setLabel('left', text='Y position (m)')
    p.setLabel('bottom', text='X position (m)')
    s = p.plot([],[],pen=None,symbol='o')
    win.show()
    tmr = QtCore.QTimer(); tmr.timeout.connect(update); tmr.start(50)
    app.exec_()
    CLIport.write(b'sensorStop\n'); CLIport.close(); Dataport.close()

import os
import sys
import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# 动态查找桌面下唯一 .cfg 文件
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
cfgs = [f for f in os.listdir(desktop) if f.lower().endswith('.cfg')]
if len(cfgs) != 1:
    print(f"请确保桌面上恰好有一个 .cfg 文件，当前找到：{cfgs}")
    sys.exit(1)
configFileName = os.path.join(desktop, cfgs[0])
print("使用配置文件：", configFileName)

# 全局变量
CLIport = None
Dataport = None
byteBuffer = np.zeros(2**15, dtype='uint8')
byteBufferLength = 0
configParameters = None

# ------------------------------------------------------------------
# 配置串口并发送配置及启动命令
def serialConfig(cfgName):
    """
    自动检测CP2105虚拟串口，分配CLI和Data口，并发送配置和启动命令
    """
    global CLIport, Dataport
    # 列出所有串口及其描述
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"Port: {p.device}, Desc: {p.description}")
    # 筛选包含 CP2105 驱动的端口
    cp2105_ports = [p for p in ports if 'CP2105' in p.description]
    if len(cp2105_ports) < 2:
        raise RuntimeError(f"未找到两个CP2105端口，发现: {[p.device for p in cp2105_ports]}")
    # 根据描述区分 Standard(命令) 和 Enhanced(数据)
    std = next((p.device for p in cp2105_ports if 'Standard' in p.description), None)
    enh = next((p.device for p in cp2105_ports if 'Enhanced' in p.description), None)
    if not std or not enh:
        # 如果无法区分，则按自然顺序分配
        std, enh = cp2105_ports[0].device, cp2105_ports[1].device
    print(f"Using CLIport={std}, Dataport={enh}")
    # 打开串口
    CLIport = serial.Serial(std, 115200)
    Dataport = serial.Serial(enh, 921600)
    # 下发配置文件所有行
    with open(cfgName, 'r') as f:
        for line in f:
            cmd = line.strip()
            CLIport.write((cmd + '
').encode())
            print(f"SENDCFG: {cmd}")
            time.sleep(0.01)
    # 启动雷达
    CLIport.write(b'sensorStart
')
    print("SEND: sensorStart")
    time.sleep(0.01)
    return CLIport, Dataport

# ------------------------------------------------------------------
# 解析配置文件，提取必要参数
def parseConfigFile(cfgName):
    lines = [l.strip() for l in open(cfgName)]
    numRxAnt, numTxAnt = 4, 3
    for ln in lines:
        parts = ln.split()
        if parts[0] == 'profileCfg':
            startFreq = float(parts[2])
            idleTime = float(parts[3])
            rampEndTime = float(parts[5])
            freqSlope = float(parts[8])
            numAdc = int(parts[10])
            adcsRound = 1
            while adcsRound < numAdc:
                adcsRound <<= 1
            digOutRate = int(parts[11])
        elif parts[0] == 'frameCfg':
            chirpStart = int(parts[1])
            chirpEnd   = int(parts[2])
            numLoops   = int(parts[3])
    numChirps = (chirpEnd - chirpStart + 1) * numLoops
    params = {}
    params['numDopplerBins']  = numChirps // numTxAnt
    params['numRangeBins']    = adcsRound
    params['rangeIdx2m']      = (3e8 * digOutRate * 1e3) / (2 * freqSlope * 1e12 * adcsRound)
    params['dopplerRes']      = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * params['numDopplerBins'] * numTxAnt)
    return params

# ------------------------------------------------------------------
# 从串口读取并解析数据包，返回点云信息
def readAndParseData18xx(Dataport, cfgParams):
    global byteBuffer, byteBufferLength
    # 常量定义
    MMWDEMO_UART_MSG_DETECTED_POINTS      = 1
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
    maxBufferSize = 2**15
    magicWord = [2,1,4,3,6,5,8,7]

    dataOK = False
    frameNumber = 0
    detObj = {}

    # 读取串口缓存
    readBytes = Dataport.read(Dataport.in_waiting or 1)
    byteVec = np.frombuffer(readBytes, dtype='uint8')
    count = len(byteVec)
    if byteBufferLength + count < maxBufferSize:
        byteBuffer[byteBufferLength: byteBufferLength+count] = byteVec
        byteBufferLength += count

    # 查找 magic word
    if byteBufferLength > len(magicWord):
        locs = np.where(byteBuffer == magicWord[0])[0]
        startIdx = [i for i in locs if np.all(byteBuffer[i:i+len(magicWord)] == magicWord)]
        if startIdx:
            idx0 = startIdx[0]
            byteBuffer[:byteBufferLength-idx0] = byteBuffer[idx0:byteBufferLength]
            byteBufferLength -= idx0
            totalLen = int(np.matmul(byteBuffer[12:16], [1,2**8,2**16,2**24]))
            if byteBufferLength >= totalLen:
                dataOK = True
                idX = 0
                idX += 20  # skip magic(8)+version(4)+pktLen(4)+platform(4)
                frameNumber = int(np.matmul(byteBuffer[idX:idX+4], [1,2**8,2**16,2**24])); idX += 4
                numDet = int(np.matmul(byteBuffer[idX:idX+4], [1,2**8,2**16,2**24])); idX += 4
                numTLV = int(np.matmul(byteBuffer[idX:idX+4], [1,2**8,2**16,2**24])); idX += 4
                for _ in range(numTLV):
                    tlvType = int(np.matmul(byteBuffer[idX:idX+4], [1,2**8,2**16,2**24])); idX += 4
                    tlvLen  = int(np.matmul(byteBuffer[idX:idX+4], [1,2**8,2**16,2**24])); idX += 4
                    if tlvType == MMWDEMO_UART_MSG_DETECTED_POINTS:
                        x = np.zeros(numDet, dtype=np.float32)
                        y = np.zeros(numDet, dtype=np.float32)
                        z = np.zeros(numDet, dtype=np.float32)
                        v = np.zeros(numDet, dtype=np.float32)
                        for idx in range(numDet):
                            x[idx] = byteBuffer[idX:idX+4].view(np.float32); idX += 4
                            y[idx] = byteBuffer[idX:idX+4].view(np.float32); idX += 4
                            z[idx] = byteBuffer[idX:idX+4].view(np.float32); idX += 4
                            v[idx] = byteBuffer[idX:idX+4].view(np.float32); idX += 4
                        detObj = {'numObj':numDet, 'x':x, 'y':y, 'z':z, 'velocity':v}
                    else:
                        idX += tlvLen
                # 清理 buffer
                byteBuffer[:byteBufferLength-totalLen] = byteBuffer[totalLen:byteBufferLength]
                byteBufferLength -= totalLen

    return dataOK, frameNumber, detObj

# ------------------------------------------------------------------
# 定时调用：更新并绘制
def update():
    dataOk, fnum, detObj = readAndParseData18xx(Dataport, configParameters)
    if dataOk and 'x' in detObj and len(detObj['x'])>0:
        s.setData(-detObj['x'], detObj['y'])
    return dataOk

# ------------------------- 主程序 -------------------------
if __name__ == '__main__':
    # 初始化
    CLIport, Dataport = serialConfig(configFileName)
    configParameters    = parseConfigFile(configFileName)

    # GUI 界面
    app = QtWidgets.QApplication([])
    pg.setConfigOption('background', 'w')
    win = pg.GraphicsLayoutWidget(title="2D scatter plot")
    p = win.addPlot()
    p.setXRange(-0.5, 0.5)
    p.setYRange(0, 1.5)
    p.setLabel('left',   text='Y position (m)')
    p.setLabel('bottom', text='X position (m)')
    s = p.plot([], [], pen=None, symbol='o')
    win.show()

    # 定时更新
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    app.exec_()

    # 退出清理
    CLIport.write(b'sensorStop\n')
    CLIport.close()
    Dataport.close()

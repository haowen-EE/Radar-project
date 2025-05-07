import os
import sys
import glob
import time
import numpy as np

# 串口与端口扫描
import serial
import serial.tools.list_ports

# 绘图
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.opengl import GLViewWidget, GLScatterPlotItem

# ─── 配置文件自动发现 ─────────────────────────────
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
cfgs = glob.glob(os.path.join(desktop, '*.cfg'))
if len(cfgs) != 1:
    print("请确保桌面上恰好有一个 .cfg 文件，当前找到：", cfgs)
    sys.exit(1)
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

    # 自动扫描 “Standard COM Port”（115200 波特）与 “Enhanced COM Port”（921600 波特）
    cli_port_name  = None
    data_port_name = None
    for p in serial.tools.list_ports.comports():
        desc = p.description.lower()
        if 'standard com port' in desc:
            cli_port_name = p.device
        elif 'enhanced com port' in desc:
            data_port_name = p.device

    if cli_port_name is None or data_port_name is None:
        ports = [(p.device, p.description) for p in serial.tools.list_ports.comports()]
        raise RuntimeError(f"无法识别 CLI/Data 端口，请检查设备管理器，当前设备列表：{ports}")

    print(f"Using CLIport={cli_port_name}, Dataport={data_port_name}")
    CLIport  = serial.Serial(cli_port_name, 115200)
    Dataport = serial.Serial(data_port_name, 921600)

    # 依次下发 cfg 文件中的每条命令
    with open(cfgName, 'r') as f:
        for line in f:
            cmd = line.strip()
            print("SENDCFG:", cmd)
            CLIport.write((cmd + '\n').encode())
            time.sleep(0.01)

    # **确保输出检测点和 Range–Doppler**
    print("SENDCFG: outputConfig 0 1 1   # TLV1: detected points")
    CLIport.write(b'outputConfig 0 1 1\n')
    time.sleep(0.01)
    print("SENDCFG: outputConfig 0 5 1   # TLV5: range-Doppler heatmap")
    CLIport.write(b'outputConfig 0 5 1\n')
    time.sleep(0.01)

    # 启动雷达采集
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

    numTxAnt = 3
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"]      = int(numChirpsPerFrame / numTxAnt)
    configParameters["numRangeBins"]        = numAdcSamplesRoundTo2
    configParameters["rangeIdxToMeters"]    = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamplesRoundTo2)
    configParameters["dopplerResolutionMps"] = 3e8 / (
        2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 *
        configParameters["numDopplerBins"] * numTxAnt)
    return configParameters

# ─── 读取并解析 TLV 数据 ─────────────────────────
def readAndParseData18xx(port, configParameters):
    global byteBuffer, byteBufferLength

    # 调试：打印串口等待字节
    waiting = port.in_waiting
    print(f"DEBUG: {waiting} bytes waiting on Dataport")

    MMWDEMO_UART_MSG_DETECTED_POINTS          = 1
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
    magicWord = [2,1,4,3,6,5,8,7]
    maxBufferSize = 2**15

    dataOK = False
    frameNumber = 0
    detObj = {}
    rdMat = None

    # 读取所有可用字节
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
                i += 8+4+4+4  # 跳过头部
                frameNumber = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24]))
                i += 4
                numObj  = int(np.dot(byteBuffer[i:i+4], [1,2**8,2**16,2**24])); i+=4
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
                        numBytes = 2 * rb * db
                        payload = byteBuffer[i:i+numBytes]; i += numBytes
                        mat = payload.view(np.int16).reshape((db, rb), order='F')
                        half = db // 2
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
        if detObj.get('numObj', 0) > 0:
            print("DEBUG: updating 3D scatter with", detObj['numObj'], "points")
            pts = np.vstack((detObj['x'], detObj['y'], detObj['z'])).T
            scatter3d.setData(pos=pts, size=0.05, color=(1,0,0,1))
        if rdMat is not None:
            print("DEBUG: updating RD plot, shape=", rdMat.shape)
            rd_img.setImage(rdMat, autoLevels=True)

# ─── 主程序入口 ─────────────────────────────────
if __name__ == '__main__':
    CLIport, Dataport  = serialConfig(configFileName)
    configParameters  = parseConfigFile(configFileName)

    app = QtWidgets.QApplication([])

    # 3D Scatter Plot 窗口
    view3d = GLViewWidget(); view3d.setWindowTitle('3D Scatter Plot'); view3d.opts['distance']=3
    scatter3d = GLScatterPlotItem(); view3d.addItem(scatter3d); view3d.show()

    # Range–Doppler Plot 窗口
    rd_win = pg.GraphicsLayoutWidget(title='Range–Doppler Plot')
    rd_plot = rd_win.addPlot(); rd_plot.setLabel('left','Doppler Bin'); rd_plot.setLabel('bottom','Range Bin')
    rd_img  = pg.ImageItem(); rd_plot.addItem(rd_img); rd_win.show()

    timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(50)
    app.exec_()

    CLIport.write(b'sensorStop\n'); CLIport.close(); Dataport.close()

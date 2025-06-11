'''

ser = serial.Serial('/dev/ttyUSB0','115200')
#ser.write(b'\xFF\xFC\x07\x10\x01\x01\x01\x01\x1B') #straight
#ser.write(b'\xFF\xFC\x07\x10\x20\x20\xDF\xDF\x15') #left turn
#ser.write(b'\xFF\xFC\x07\x10\xDF\x20\x20\xDF\x15') #left parallel
#ser.write(b'\xFF\xFC\x07\x10\x20\xDF\xDF\x20\x15') #right parallel
#ser.write(b'\xFF\xFC\x07\x10\x00\xDF\x00\x20\x16') #right front turn

ser.write(b'\xFF\xFC\x07\x10\xDF\x20\x20\x20\x56')
time.sleep(60)
ser.write(b'\xFF\xFC\x06\x11\x00\x00\x00\x17')

'''




import serial
import time
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5 import QtCore
from PyQt5.QtCore import *


machineArm = {'ID1':2064, 'ID2':960, 'ID3':3040, 'ID4':3040, 'ID5':1472, 'ID6':1280, 'Time':0}

def machineArmCal():
        
        global machineArm
        
        portx="/dev/ttyUSB0"
        bps=115200
        timex=None
        ser=serial.Serial(portx,bps,timeout=timex)
        
        id1 = machineArm['ID1']
        id2 = machineArm['ID2'] 
        id3 = machineArm['ID3'] 
        id4 = machineArm['ID4'] 
        id5 = machineArm['ID5'] 
        id6 = machineArm['ID6'] 
        time = machineArm['Time'] 
        
        id1 = str(format(id1, '04X'))
        id2 = str(format(id2, '04X'))
        id3 = str(format(id3, '04X'))
        id4 = str(format(id4, '04X'))
        id5 = str(format(id5, '04X'))
        id6 = str(format(id6, '04X'))
        time = str(format(time, '04X'))
        
        id1h = id1[:2]
        id1l = id1[2:]
        
        id2h = id2[:2]
        id2l = id2[2:]
        
        id3h = id3[:2]
        id3l = id3[2:]
        
        id4h = id4[:2]
        id4l = id4[2:]
        
        id5h = id5[:2]
        id5l = id5[2:]
        
        id6h = id6[:2]
        id6l = id6[2:]
        
        timeh = time[:2]
        timel = time[2:]
        
        m1 = int(id1l, 16)
        m2 = int(id1h, 16)
        m3 = int(id2l, 16)
        m4 = int(id2h, 16)
        m5 = int(id3l, 16)
        m6 = int(id3h, 16)
        m7 = int(id4l, 16)
        m8 = int(id4h, 16)
        m9 = int(id5l, 16)
        m10 = int(id5h, 16)
        m11 = int(id6l, 16)
        m12 = int(id6h, 16)
        m13 = int(timel, 16)
        m14 = int(timeh, 16)


        checksum = (0x11 + 0x23 + m1 + m2 + m3 + m4 + m5 + m6 + m7 + m8 + m9 + m10 + m11 + m12 + m13 + m14) % 256
        
        checksum = str(format(checksum, '02X'))



        
        command = '\\xFF\\xFC\\x11\\x23' + '\\x' + id1l + '\\x' + id1h + '\\x' + id2l + '\\x' + id2h + '\\x' + id3l + '\\x' + id3h + '\\x' + id4l + '\\x' + id4h + '\\x' + id5l + '\\x' + id5h +'\\x' + id6l + '\\x' + id6h + '\\x' + timel + '\\x' + timeh + '\\x' + checksum
        
        '''
        print(command)
        
        command = command.encode('utf-8') 
        
        print(command)
        '''
        
        print(command)
        
        hexNumbers = command.split("\\x")
        decimalNumbers = [int(hexNumber, 16) for hexNumber in hexNumbers if hexNumber != ""]
        byteStream = bytes(decimalNumbers)
        
        
        ser.write(byteStream)

class Window(QWidget):

    
    
    
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 300, 200)
        self.setFixedWidth(300)
        self.setFixedHeight(200)
        self.setWindowTitle('Driver')
        self.show()

    

    # 检测键盘回车按键
    def keyPressEvent(self,event):
        portx="/dev/ttyUSB0"
        bps=115200
        timex=None
        ser=serial.Serial(portx,bps,timeout=timex)
        
        global machineArm

        if (event.key() == Qt.Key_Up):
            ser.write(b'\xFF\xFC\x07\x10\x22\x20\x20\x20\x99')
            #ser.write(b'\xFF\xFC\x07\x10\x09\x10\x10\x10\x50')
            #ser.write(b'\xFF\xFC\x07\x10\x64\x64\x64\x64\xA7')
        if (event.key() == Qt.Key_Left):
            #ser.write(b'\xFF\xFC\x07\x10\xDF\x20\x20\x20\x56') 
            ser.write(b'\xFF\xFC\x07\x10\xF7\x20\x10\x20\x5E') 
        if (event.key() == Qt.Key_Right):
            ser.write(b'\xFF\xFC\x07\x10\x20\x20\xDF\x20\x56')
        if (event.key() == Qt.Key_Down):
            ser.write(b'\xFF\xFC\x07\x10\xDF\xE7\xDF\xE7\xA3')
        if (event.key() == Qt.Key_Q):
            ser.write(b'\xFF\xFC\x07\x10\xDF\xDF\x20\x20\x15')
        if (event.key() == Qt.Key_E):
            ser.write(b'\xFF\xFC\x07\x10\x20\x20\xDF\xDF\x15')
        if (event.key() == Qt.Key_A):
            ser.write(b'\xFF\xFC\x07\x10\xDF\x20\x20\xDF\x15')
        if (event.key() == Qt.Key_D):
            ser.write(b'\xFF\xFC\x07\x10\x20\xDF\xDF\x20\x15')
        if (event.key() == Qt.Key_C):
            ser.write(b'\xFF\xFC\x07\x10\x20\x00\x00\x20\x57')
        if (event.key() == Qt.Key_Z):
            ser.write(b'\xFF\xFC\x07\x10\x00\x20\x20\x00\x57')
        if (event.key() == Qt.Key_P):
            if machineArm['ID6'] > 3232:
                print("超出限位")
            else:
                i = machineArm['ID6'] + 32
                machineArm['ID6'] = i
                machineArmCal()
        if (event.key() == Qt.Key_O):
            if machineArm['ID6'] < 1280:
                print("低于限位")
            else:
                i = machineArm['ID6'] - 32
                machineArm['ID6'] = i
                machineArmCal()
        if (event.key() == Qt.Key_N):
            if machineArm['ID5'] > 2369:
                print("超出限位")
            else:
                i = machineArm['ID5'] + 32
                machineArm['ID5'] = i
                machineArmCal()
        if (event.key() == Qt.Key_V):
            if machineArm['ID5'] < 616:
                print("低于限位")
            else:
                i = machineArm['ID5'] - 32
                machineArm['ID5'] = i
                machineArmCal()
        if (event.key() == Qt.Key_H):
            if machineArm['ID4'] > 3040:
                print("超出限位")
            else:
                i = machineArm['ID4'] + 32
                machineArm['ID4'] = i
                machineArmCal()
        if (event.key() == Qt.Key_F):
           if machineArm['ID4'] < 1920:
               print("低于限位")
           else:
               i = machineArm['ID4'] - 32
               machineArm['ID4'] = i
               machineArmCal()
        if (event.key() == Qt.Key_Y):
            if machineArm['ID3'] > 3040:
                print("超出限位")
            else:
                i = machineArm['ID3'] + 32
                machineArm['ID3'] = i
                machineArmCal()
        if (event.key() == Qt.Key_R):
            if machineArm['ID3'] < 1984:
                print("低于限位")
            else:
                i = machineArm['ID3'] - 32
                machineArm['ID3'] = i
                machineArmCal()
        if (event.key() == Qt.Key_L):
            if machineArm['ID2'] > 3104:
                print("超出限位")
            else:
                i = machineArm['ID2'] + 32
                machineArm['ID2'] = i
                machineArmCal()
        if (event.key() == Qt.Key_J):
            if machineArm['ID2'] < 960:
                print("低于限位")
            else:
                i = machineArm['ID2'] - 32
                machineArm['ID2'] = i
                machineArmCal()
        if (event.key() == Qt.Key_T):
            if machineArm['ID1'] > 3728:
                print("超出限位")
            else:
                i = machineArm['ID1'] + 32
                machineArm['ID1'] = i
                machineArmCal()
        if (event.key() == Qt.Key_G):
            if machineArm['ID1'] < 400:
                print("低于限位")
            else:
                i = machineArm['ID1'] - 32
                machineArm['ID1'] = i
                machineArmCal()
        if (event.key() == Qt.Key_B):
            ser.write(b'\xFF\xFC\x11\x23\x10\x08\xC0\x03\xE0\x0B\xE0\x0B\xC0\x05\x00\x05\x00\x00\xAF')
            machineArm = {'ID1':2064, 'ID2':960, 'ID3':3040, 'ID4':3040, 'ID5':1472, 'ID6':1280, 'Time':0}
        if (event.key() == Qt.Key_M):
            ser.write(b'\xFF\xFC\x11\x23\x10\x08\xC0\x03\xE0\x0B\xE0\x0B\xC0\x05\xC0\x0C\x00\x00\x76')
            machineArm = {'ID1':2064, 'ID2':960, 'ID3':3040, 'ID4':3040, 'ID5':1472, 'ID6':3232, 'Time':0}
            
            
            

    def keyReleaseEvent(self, event):      #重载
        portx="/dev/ttyUSB0"
        bps=115200
        #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        timex=None
        ser=serial.Serial(portx,bps,timeout=timex)
        if event.key() == Qt.Key_Up:     
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_Left:     
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_Right:     
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_Down:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_Q:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_E:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_A:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_D:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_Z:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
        if event.key() == Qt.Key_C:
            if event.isAutoRepeat():       #按钮按下状态，为True
                pass
            else:                          #按钮释放状态，为False
                ser.write(b'\xFF\xFC\x07\x10\x00\x00\x00\x00\x17')#写数据 ###STOP
                

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    sys.exit(app.exec_())
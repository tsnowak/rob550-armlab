# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1000, 810)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.videoFrame = QtGui.QLabel(self.centralwidget)
        self.videoFrame.setGeometry(QtCore.QRect(310, 30, 640, 480))
        self.videoFrame.setObjectName(_fromUtf8("videoFrame"))
        self.SliderFrame = QtGui.QGroupBox(self.centralwidget)
        self.SliderFrame.setGeometry(QtCore.QRect(310, 570, 531, 201))
        self.SliderFrame.setObjectName(_fromUtf8("SliderFrame"))
        self.layoutWidget = QtGui.QWidget(self.SliderFrame)
        self.layoutWidget.setGeometry(QtCore.QRect(0, 20, 65, 136))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.BLabelS = QtGui.QLabel(self.layoutWidget)
        self.BLabelS.setObjectName(_fromUtf8("BLabelS"))
        self.verticalLayout.addWidget(self.BLabelS, QtCore.Qt.AlignRight)
        self.SLabelS = QtGui.QLabel(self.layoutWidget)
        self.SLabelS.setObjectName(_fromUtf8("SLabelS"))
        self.verticalLayout.addWidget(self.SLabelS, QtCore.Qt.AlignRight)
        self.ELabelS = QtGui.QLabel(self.layoutWidget)
        self.ELabelS.setObjectName(_fromUtf8("ELabelS"))
        self.verticalLayout.addWidget(self.ELabelS, QtCore.Qt.AlignRight)
        self.WLabelS = QtGui.QLabel(self.layoutWidget)
        self.WLabelS.setObjectName(_fromUtf8("WLabelS"))
        self.verticalLayout.addWidget(self.WLabelS, QtCore.Qt.AlignRight)
        self.layoutWidget1 = QtGui.QWidget(self.SliderFrame)
        self.layoutWidget1.setGeometry(QtCore.QRect(70, 20, 371, 136))
        self.layoutWidget1.setObjectName(_fromUtf8("layoutWidget1"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.sldrBase = QtGui.QSlider(self.layoutWidget1)
        self.sldrBase.setMinimum(-179)
        self.sldrBase.setMaximum(180)
        self.sldrBase.setOrientation(QtCore.Qt.Horizontal)
        self.sldrBase.setObjectName(_fromUtf8("sldrBase"))
        self.verticalLayout_2.addWidget(self.sldrBase)
        self.sldrShoulder = QtGui.QSlider(self.layoutWidget1)
        self.sldrShoulder.setMinimum(-124)
        self.sldrShoulder.setMaximum(125)
        self.sldrShoulder.setOrientation(QtCore.Qt.Horizontal)
        self.sldrShoulder.setObjectName(_fromUtf8("sldrShoulder"))
        self.verticalLayout_2.addWidget(self.sldrShoulder)
        self.sldrElbow = QtGui.QSlider(self.layoutWidget1)
        self.sldrElbow.setMinimum(-123)
        self.sldrElbow.setMaximum(125)
        self.sldrElbow.setOrientation(QtCore.Qt.Horizontal)
        self.sldrElbow.setObjectName(_fromUtf8("sldrElbow"))
        self.verticalLayout_2.addWidget(self.sldrElbow)
        self.sldrWrist = QtGui.QSlider(self.layoutWidget1)
        self.sldrWrist.setMinimum(-125)
        self.sldrWrist.setMaximum(128)
        self.sldrWrist.setOrientation(QtCore.Qt.Horizontal)
        self.sldrWrist.setObjectName(_fromUtf8("sldrWrist"))
        self.verticalLayout_2.addWidget(self.sldrWrist)
        self.layoutWidget2 = QtGui.QWidget(self.SliderFrame)
        self.layoutWidget2.setGeometry(QtCore.QRect(462, 20, 51, 137))
        self.layoutWidget2.setObjectName(_fromUtf8("layoutWidget2"))
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.layoutWidget2)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.rdoutBase = QtGui.QLabel(self.layoutWidget2)
        self.rdoutBase.setObjectName(_fromUtf8("rdoutBase"))
        self.verticalLayout_6.addWidget(self.rdoutBase)
        self.rdoutShoulder = QtGui.QLabel(self.layoutWidget2)
        self.rdoutShoulder.setObjectName(_fromUtf8("rdoutShoulder"))
        self.verticalLayout_6.addWidget(self.rdoutShoulder)
        self.rdoutElbow = QtGui.QLabel(self.layoutWidget2)
        self.rdoutElbow.setObjectName(_fromUtf8("rdoutElbow"))
        self.verticalLayout_6.addWidget(self.rdoutElbow)
        self.rdoutWrist = QtGui.QLabel(self.layoutWidget2)
        self.rdoutWrist.setObjectName(_fromUtf8("rdoutWrist"))
        self.verticalLayout_6.addWidget(self.rdoutWrist)
        self.sldrGrip1 = QtGui.QSlider(self.SliderFrame)
        self.sldrGrip1.setGeometry(QtCore.QRect(78, 160, 133, 29))
        self.sldrGrip1.setMinimum(-179)
        self.sldrGrip1.setMaximum(180)
        self.sldrGrip1.setOrientation(QtCore.Qt.Horizontal)
        self.sldrGrip1.setObjectName(_fromUtf8("sldrGrip1"))
        self.sldrGrip2 = QtGui.QSlider(self.SliderFrame)
        self.sldrGrip2.setGeometry(QtCore.QRect(300, 160, 130, 29))
        self.sldrGrip2.setMinimum(-179)
        self.sldrGrip2.setMaximum(180)
        self.sldrGrip2.setOrientation(QtCore.Qt.Horizontal)
        self.sldrGrip2.setObjectName(_fromUtf8("sldrGrip2"))
        self.G1LableS = QtGui.QLabel(self.SliderFrame)
        self.G1LableS.setGeometry(QtCore.QRect(25, 165, 41, 17))
        self.G1LableS.setObjectName(_fromUtf8("G1LableS"))
        self.rdoutGrip1 = QtGui.QLabel(self.SliderFrame)
        self.rdoutGrip1.setGeometry(QtCore.QRect(216, 160, 49, 28))
        self.rdoutGrip1.setObjectName(_fromUtf8("rdoutGrip1"))
        self.rdoutGrip2 = QtGui.QLabel(self.SliderFrame)
        self.rdoutGrip2.setGeometry(QtCore.QRect(436, 160, 49, 28))
        self.rdoutGrip2.setObjectName(_fromUtf8("rdoutGrip2"))
        self.sldrMaxTorque = QtGui.QSlider(self.centralwidget)
        self.sldrMaxTorque.setGeometry(QtCore.QRect(850, 610, 30, 115))
        self.sldrMaxTorque.setMaximum(100)
        self.sldrMaxTorque.setProperty("value", 70)
        self.sldrMaxTorque.setOrientation(QtCore.Qt.Vertical)
        self.sldrMaxTorque.setObjectName(_fromUtf8("sldrMaxTorque"))
        self.TqLabel = QtGui.QLabel(self.centralwidget)
        self.TqLabel.setGeometry(QtCore.QRect(840, 580, 52, 20))
        self.TqLabel.setObjectName(_fromUtf8("TqLabel"))
        self.sldrSpeed = QtGui.QSlider(self.centralwidget)
        self.sldrSpeed.setGeometry(QtCore.QRect(920, 610, 29, 115))
        self.sldrSpeed.setMaximum(100)
        self.sldrSpeed.setProperty("value", 40)
        self.sldrSpeed.setOrientation(QtCore.Qt.Vertical)
        self.sldrSpeed.setObjectName(_fromUtf8("sldrSpeed"))
        self.SpLabel = QtGui.QLabel(self.centralwidget)
        self.SpLabel.setGeometry(QtCore.QRect(910, 580, 41, 21))
        self.SpLabel.setObjectName(_fromUtf8("SpLabel"))
        self.OutputFrame = QtGui.QFrame(self.centralwidget)
        self.OutputFrame.setGeometry(QtCore.QRect(20, 10, 221, 311))
        self.OutputFrame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.OutputFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.OutputFrame.setObjectName(_fromUtf8("OutputFrame"))
        self.JointCoordLabel = QtGui.QLabel(self.OutputFrame)
        self.JointCoordLabel.setGeometry(QtCore.QRect(30, 10, 141, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.JointCoordLabel.setFont(font)
        self.JointCoordLabel.setObjectName(_fromUtf8("JointCoordLabel"))
        self.WorldCoordLabel = QtGui.QLabel(self.OutputFrame)
        self.WorldCoordLabel.setGeometry(QtCore.QRect(20, 160, 161, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.WorldCoordLabel.setFont(font)
        self.WorldCoordLabel.setObjectName(_fromUtf8("WorldCoordLabel"))
        self.layoutWidget_2 = QtGui.QWidget(self.OutputFrame)
        self.layoutWidget_2.setGeometry(QtCore.QRect(80, 30, 131, 120))
        self.layoutWidget_2.setObjectName(_fromUtf8("layoutWidget_2"))
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.layoutWidget_2)
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.rdoutBaseJC = QtGui.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutBaseJC.setFont(font)
        self.rdoutBaseJC.setObjectName(_fromUtf8("rdoutBaseJC"))
        self.verticalLayout_8.addWidget(self.rdoutBaseJC, QtCore.Qt.AlignLeft)
        self.rdoutShoulderJC = QtGui.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutShoulderJC.setFont(font)
        self.rdoutShoulderJC.setObjectName(_fromUtf8("rdoutShoulderJC"))
        self.verticalLayout_8.addWidget(self.rdoutShoulderJC, QtCore.Qt.AlignLeft)
        self.rdoutElbowJC = QtGui.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutElbowJC.setFont(font)
        self.rdoutElbowJC.setObjectName(_fromUtf8("rdoutElbowJC"))
        self.verticalLayout_8.addWidget(self.rdoutElbowJC, QtCore.Qt.AlignLeft)
        self.rdoutWristJC = QtGui.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutWristJC.setFont(font)
        self.rdoutWristJC.setObjectName(_fromUtf8("rdoutWristJC"))
        self.verticalLayout_8.addWidget(self.rdoutWristJC, QtCore.Qt.AlignLeft)
        self.layoutWidget_3 = QtGui.QWidget(self.OutputFrame)
        self.layoutWidget_3.setGeometry(QtCore.QRect(10, 30, 66, 120))
        self.layoutWidget_3.setObjectName(_fromUtf8("layoutWidget_3"))
        self.verticalLayout_9 = QtGui.QVBoxLayout(self.layoutWidget_3)
        self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
        self.BLabel = QtGui.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.BLabel.setFont(font)
        self.BLabel.setObjectName(_fromUtf8("BLabel"))
        self.verticalLayout_9.addWidget(self.BLabel, QtCore.Qt.AlignRight)
        self.SLabel = QtGui.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.SLabel.setFont(font)
        self.SLabel.setObjectName(_fromUtf8("SLabel"))
        self.verticalLayout_9.addWidget(self.SLabel, QtCore.Qt.AlignRight)
        self.ELabel = QtGui.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.ELabel.setFont(font)
        self.ELabel.setObjectName(_fromUtf8("ELabel"))
        self.verticalLayout_9.addWidget(self.ELabel, QtCore.Qt.AlignRight)
        self.WLabel = QtGui.QLabel(self.layoutWidget_3)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.WLabel.setFont(font)
        self.WLabel.setObjectName(_fromUtf8("WLabel"))
        self.verticalLayout_9.addWidget(self.WLabel, QtCore.Qt.AlignRight)
        self.layoutWidget_4 = QtGui.QWidget(self.OutputFrame)
        self.layoutWidget_4.setGeometry(QtCore.QRect(80, 179, 131, 121))
        self.layoutWidget_4.setObjectName(_fromUtf8("layoutWidget_4"))
        self.verticalLayout_12 = QtGui.QVBoxLayout(self.layoutWidget_4)
        self.verticalLayout_12.setObjectName(_fromUtf8("verticalLayout_12"))
        self.rdoutX = QtGui.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutX.setFont(font)
        self.rdoutX.setObjectName(_fromUtf8("rdoutX"))
        self.verticalLayout_12.addWidget(self.rdoutX, QtCore.Qt.AlignLeft)
        self.rdoutY = QtGui.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutY.setFont(font)
        self.rdoutY.setObjectName(_fromUtf8("rdoutY"))
        self.verticalLayout_12.addWidget(self.rdoutY, QtCore.Qt.AlignLeft)
        self.rdoutZ = QtGui.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutZ.setFont(font)
        self.rdoutZ.setObjectName(_fromUtf8("rdoutZ"))
        self.verticalLayout_12.addWidget(self.rdoutZ, QtCore.Qt.AlignLeft)
        self.rdoutT = QtGui.QLabel(self.layoutWidget_4)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.rdoutT.setFont(font)
        self.rdoutT.setObjectName(_fromUtf8("rdoutT"))
        self.verticalLayout_12.addWidget(self.rdoutT, QtCore.Qt.AlignLeft)
        self.layoutWidget_5 = QtGui.QWidget(self.OutputFrame)
        self.layoutWidget_5.setGeometry(QtCore.QRect(11, 180, 66, 121))
        self.layoutWidget_5.setObjectName(_fromUtf8("layoutWidget_5"))
        self.verticalLayout_13 = QtGui.QVBoxLayout(self.layoutWidget_5)
        self.verticalLayout_13.setObjectName(_fromUtf8("verticalLayout_13"))
        self.XLabel = QtGui.QLabel(self.layoutWidget_5)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.XLabel.setFont(font)
        self.XLabel.setObjectName(_fromUtf8("XLabel"))
        self.verticalLayout_13.addWidget(self.XLabel, QtCore.Qt.AlignRight)
        self.YLabel = QtGui.QLabel(self.layoutWidget_5)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.YLabel.setFont(font)
        self.YLabel.setObjectName(_fromUtf8("YLabel"))
        self.verticalLayout_13.addWidget(self.YLabel, QtCore.Qt.AlignRight)
        self.ZLabel = QtGui.QLabel(self.layoutWidget_5)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.ZLabel.setFont(font)
        self.ZLabel.setObjectName(_fromUtf8("ZLabel"))
        self.verticalLayout_13.addWidget(self.ZLabel, QtCore.Qt.AlignRight)
        self.TLabel = QtGui.QLabel(self.layoutWidget_5)
        self.TLabel.setEnabled(True)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(18)
        self.TLabel.setFont(font)
        self.TLabel.setObjectName(_fromUtf8("TLabel"))
        self.verticalLayout_13.addWidget(self.TLabel, QtCore.Qt.AlignRight)
        self.rdoutTorq = QtGui.QLabel(self.centralwidget)
        self.rdoutTorq.setGeometry(QtCore.QRect(854, 732, 50, 28))
        self.rdoutTorq.setObjectName(_fromUtf8("rdoutTorq"))
        self.rdoutSpeed = QtGui.QLabel(self.centralwidget)
        self.rdoutSpeed.setGeometry(QtCore.QRect(924, 732, 49, 28))
        self.rdoutSpeed.setObjectName(_fromUtf8("rdoutSpeed"))
        self.layoutWidget3 = QtGui.QWidget(self.centralwidget)
        self.layoutWidget3.setGeometry(QtCore.QRect(20, 330, 221, 410))
        self.layoutWidget3.setObjectName(_fromUtf8("layoutWidget3"))
        self.Group2 = QtGui.QVBoxLayout(self.layoutWidget3)
        self.Group2.setMargin(10)
        self.Group2.setObjectName(_fromUtf8("Group2"))
        self.btnUser1 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser1.setObjectName(_fromUtf8("btnUser1"))
        self.Group2.addWidget(self.btnUser1)
        self.btnUser2 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser2.setObjectName(_fromUtf8("btnUser2"))
        self.Group2.addWidget(self.btnUser2)
        self.btnUser3 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser3.setObjectName(_fromUtf8("btnUser3"))
        self.Group2.addWidget(self.btnUser3)
        self.btnUser4 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser4.setObjectName(_fromUtf8("btnUser4"))
        self.Group2.addWidget(self.btnUser4)
        self.btnUser5 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser5.setObjectName(_fromUtf8("btnUser5"))
        self.Group2.addWidget(self.btnUser5)
        self.btnUser6 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser6.setObjectName(_fromUtf8("btnUser6"))
        self.Group2.addWidget(self.btnUser6)
        self.btnUser7 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser7.setObjectName(_fromUtf8("btnUser7"))
        self.Group2.addWidget(self.btnUser7)
        self.btnUser8 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser8.setObjectName(_fromUtf8("btnUser8"))
        self.Group2.addWidget(self.btnUser8)
        self.btnUser9 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser9.setObjectName(_fromUtf8("btnUser9"))
        self.Group2.addWidget(self.btnUser9)
        self.btnUser10 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser10.setAutoRepeatDelay(300)
        self.btnUser10.setObjectName(_fromUtf8("btnUser10"))
        self.Group2.addWidget(self.btnUser10)
        self.btnUser11 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser11.setAutoRepeatDelay(300)
        self.btnUser11.setObjectName(_fromUtf8("btnUser11"))
        self.Group2.addWidget(self.btnUser11)
        self.btnUser12 = QtGui.QPushButton(self.layoutWidget3)
        self.btnUser12.setAutoRepeatDelay(300)
        self.btnUser12.setObjectName(_fromUtf8("btnUser12"))
        self.Group2.addWidget(self.btnUser12)
        self.VidFrame = QtGui.QFrame(self.centralwidget)
        self.VidFrame.setGeometry(QtCore.QRect(310, 30, 640, 480))
        self.VidFrame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.VidFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.VidFrame.setObjectName(_fromUtf8("VidFrame"))
        self.PixelCoordLabel = QtGui.QLabel(self.centralwidget)
        self.PixelCoordLabel.setGeometry(QtCore.QRect(310, 520, 181, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.PixelCoordLabel.setFont(font)
        self.PixelCoordLabel.setObjectName(_fromUtf8("PixelCoordLabel"))
        self.PixelCoordLabel_2 = QtGui.QLabel(self.centralwidget)
        self.PixelCoordLabel_2.setGeometry(QtCore.QRect(610, 520, 231, 17))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.PixelCoordLabel_2.setFont(font)
        self.PixelCoordLabel_2.setObjectName(_fromUtf8("PixelCoordLabel_2"))
        self.rdoutMousePixels = QtGui.QLabel(self.centralwidget)
        self.rdoutMousePixels.setGeometry(QtCore.QRect(500, 516, 101, 25))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.rdoutMousePixels.setFont(font)
        self.rdoutMousePixels.setTextFormat(QtCore.Qt.AutoText)
        self.rdoutMousePixels.setObjectName(_fromUtf8("rdoutMousePixels"))
        self.rdoutMouseWorld = QtGui.QLabel(self.centralwidget)
        self.rdoutMouseWorld.setGeometry(QtCore.QRect(848, 516, 101, 25))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Ubuntu Mono"))
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.rdoutMouseWorld.setFont(font)
        self.rdoutMouseWorld.setTextFormat(QtCore.Qt.AutoText)
        self.rdoutMouseWorld.setObjectName(_fromUtf8("rdoutMouseWorld"))
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(2, 760, 971, 26))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.rdoutStatus = QtGui.QLabel(self.groupBox)
        self.rdoutStatus.setGeometry(QtCore.QRect(70, 0, 905, 18))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.rdoutStatus.setFont(font)
        self.rdoutStatus.setTextFormat(QtCore.Qt.AutoText)
        self.rdoutStatus.setObjectName(_fromUtf8("rdoutStatus"))
        self.G2lableS = QtGui.QLabel(self.centralwidget)
        self.G2lableS.setGeometry(QtCore.QRect(565, 735, 41, 17))
        self.G2lableS.setObjectName(_fromUtf8("G2lableS"))
        self.videoFrame.raise_()
        self.SliderFrame.raise_()
        self.sldrMaxTorque.raise_()
        self.TqLabel.raise_()
        self.sldrSpeed.raise_()
        self.SpLabel.raise_()
        self.OutputFrame.raise_()
        self.rdoutTorq.raise_()
        self.rdoutSpeed.raise_()
        self.layoutWidget.raise_()
        self.VidFrame.raise_()
        self.PixelCoordLabel.raise_()
        self.PixelCoordLabel_2.raise_()
        self.rdoutMousePixels.raise_()
        self.rdoutMouseWorld.raise_()
        self.G2lableS.raise_()
        self.groupBox.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Rexarm GUI", None))
        self.videoFrame.setText(_translate("MainWindow", "TextLabel", None))
        self.SliderFrame.setTitle(_translate("MainWindow", "Joint Sliders", None))
        self.BLabelS.setText(_translate("MainWindow", "Base", None))
        self.SLabelS.setText(_translate("MainWindow", "Shoulder", None))
        self.ELabelS.setText(_translate("MainWindow", "Elbow", None))
        self.WLabelS.setText(_translate("MainWindow", "Wrist", None))
        self.rdoutBase.setText(_translate("MainWindow", "0", None))
        self.rdoutShoulder.setText(_translate("MainWindow", "0", None))
        self.rdoutElbow.setText(_translate("MainWindow", "0", None))
        self.rdoutWrist.setText(_translate("MainWindow", "0", None))
        self.G1LableS.setText(_translate("MainWindow", "Grip 1", None))
        self.rdoutGrip1.setText(_translate("MainWindow", "0", None))
        self.rdoutGrip2.setText(_translate("MainWindow", "0", None))
        self.TqLabel.setText(_translate("MainWindow", "Torque", None))
        self.SpLabel.setText(_translate("MainWindow", "Speed", None))
        self.JointCoordLabel.setText(_translate("MainWindow", "Joint Coordinates", None))
        self.WorldCoordLabel.setText(_translate("MainWindow", "End Effector Location", None))
        self.rdoutBaseJC.setText(_translate("MainWindow", "0", None))
        self.rdoutShoulderJC.setText(_translate("MainWindow", "0", None))
        self.rdoutElbowJC.setText(_translate("MainWindow", "0", None))
        self.rdoutWristJC.setText(_translate("MainWindow", "0", None))
        self.BLabel.setText(_translate("MainWindow", "B:", None))
        self.SLabel.setText(_translate("MainWindow", "S:", None))
        self.ELabel.setText(_translate("MainWindow", "E:", None))
        self.WLabel.setText(_translate("MainWindow", "W:", None))
        self.rdoutX.setText(_translate("MainWindow", "0", None))
        self.rdoutY.setText(_translate("MainWindow", "0", None))
        self.rdoutZ.setText(_translate("MainWindow", "0", None))
        self.rdoutT.setText(_translate("MainWindow", "0", None))
        self.XLabel.setText(_translate("MainWindow", "X:", None))
        self.YLabel.setText(_translate("MainWindow", "Y:", None))
        self.ZLabel.setText(_translate("MainWindow", "Z:", None))
        self.TLabel.setText(_translate("MainWindow", "Phi:", None))
        self.rdoutTorq.setText(_translate("MainWindow", "0", None))
        self.rdoutSpeed.setText(_translate("MainWindow", "0", None))
        self.btnUser1.setText(_translate("MainWindow", "USER 1", None))
        self.btnUser2.setText(_translate("MainWindow", "USER 2", None))
        self.btnUser3.setText(_translate("MainWindow", "USER 3", None))
        self.btnUser4.setText(_translate("MainWindow", "USER 4", None))
        self.btnUser5.setText(_translate("MainWindow", "USER 5", None))
        self.btnUser6.setText(_translate("MainWindow", "USER 6", None))
        self.btnUser7.setText(_translate("MainWindow", "USER 7", None))
        self.btnUser8.setText(_translate("MainWindow", "USER 8", None))
        self.btnUser9.setText(_translate("MainWindow", "USER 9", None))
        self.btnUser10.setText(_translate("MainWindow", "USER 10", None))
        self.btnUser11.setText(_translate("MainWindow", "USER 11", None))
        self.btnUser12.setText(_translate("MainWindow", "USER 12", None))
        self.PixelCoordLabel.setText(_translate("MainWindow", "Pixel Mouse Coordinates", None))
        self.PixelCoordLabel_2.setText(_translate("MainWindow", "World Mouse Coordinates [mm]", None))
        self.rdoutMousePixels.setText(_translate("MainWindow", "(-,-)", None))
        self.rdoutMouseWorld.setText(_translate("MainWindow", "(-,-)", None))
        self.groupBox.setTitle(_translate("MainWindow", "STATUS: ", None))
        self.rdoutStatus.setText(_translate("MainWindow", "Waiting for Inputs", None))
        self.G2lableS.setText(_translate("MainWindow", "Grip 2", None))


import sys
import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm

from video import Video

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
PI = 3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510
 
class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    It contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video(cv2.VideoCapture(0))

        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

        """ 
        Video Function 
        Creates a timer and calls play() function 
        according to the given time delay (27mm) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(27)
       
        """ 
        LCM Arm Feedback
        Creates a timer to call LCM handler continuously
        No delay implemented. Reads all time 
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()

        """ 
        Connect Sliders to Function
        LAB TASK: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        ## TODO: IMPLEMENT GRIP VALUE CONTROLS ##
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)

        """ Commands the arm as the arm initialize to 0,0,0,0 angles """
        self.sliderChange() 
        
        """ Connect Buttons to Functions 
        LAB TASK: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Affine Calibration")
        self.ui.btnUser1.clicked.connect(self.affine_cal)
        self.ui.btnUser2.setText("STEP1: Reset Position")
        self.ui.btnUser2.clicked.connect(self.iResetPosition)
        self.ui.btnUser3.clicked.connect(self.iResetTorqueAndSpeed)
        self.ui.btnUser3.setText("STEP2: Reset Torque and Speed")
        self.ui.btnUser4.setText("STEP3: Train Begin")
        self.ui.btnUser4.clicked.connect(self.iTrainBegin)
        self.ui.btnUser5.setText("STEP4(r): GetWayPoint")
        self.ui.btnUser5.clicked.connect(self.iGetWayPoint)
        self.ui.btnUser5.setEnabled(False)
        self.ui.btnUser6.setText("STEP5: Stop Recording")
        self.ui.btnUser6.clicked.connect(self.iTrainStop)
        self.ui.btnUser6.setEnabled(False)
        self.ui.btnUser7.clicked.connect(self.iReplayBegin)
        self.ui.btnUser7.setText("Replay")
        self.ui.btnUser10.setText("PlayStop")
        self.ui.btnUser10.clicked.connect(self.iReplayStop)

    def play(self):
        """ 
        Play Funtion
        Continuously called by GUI 
        """

        """ Renders the Video Frame """
        try:
            self.video.captureNextFrame()
            self.video.blobDetector()
            self.ui.videoFrame.setPixmap(
                self.video.convertFrame())
            self.ui.videoFrame.setScaledContents(True)
        except TypeError:
            print "No frame"
        
        """ 
        Update GUI Joint Coordinates Labels
        LAB TASK: include the other slider labels 
        """
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3]*R2D)))

        """ 
        Mouse position presentation in GUI
        TO DO: after getting affine calibration make the apprriate label
        to present the value of mouse position in world coordinates 
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f)" % (x,y))
            if (self.video.aff_flag == 2):
                """ TO DO Here is where affine calibration must be used """
                self.ui.rdoutMouseWorld.setText("(-,-)")
            else:
                self.ui.rdoutMouseWorld.setText("(-,-)")

        
        """
        Set button avalibity.
        """

        self.iSetButtonAbility()

        """ 
        Updates status label when rexarm playback is been executed.
        This will be extended to includ eother appropriate messages
        """ 

        if(self.rex.plan_status == 1):
            self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
                                    %(self.rex.wpt_number + 1))

        """###############################################
        Frank Added Here
        ###############################################"""

        if (self.rex.plan_status == 2):
            self.iTrain_AddOneWay()

        if (self.rex.plan_status == 5):
            self.iReplay_PlayOneWay()



    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position 
        TO DO: Implement for the other sliders
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        self.rex.max_torque = self.ui.sldrMaxTorque.value()/100.0
        self.rex.speed = self.ui.sldrSpeed.value()/100.0
        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R

        self.rex.cmd_publish()

    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for 
        affine calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y
       
        """ If affine calibration is been performed """
        if (self.video.aff_flag == 1):
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),
                                                                 (y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1

            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                      %(self.video.mouse_click_id + 1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            LAB TASK: Change this code to use your affine calibration routine
            and NOT openCV pre-programmed function as it is done now.
            """
            if(self.video.mouse_click_id == self.video.aff_npoints):
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.aff_flag = 2
                self.video.mouse_click_id = 0
                
                """ Perform affine calibration with OpenCV """
                self.video.aff_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord,
                                        self.video.real_coord)
            
                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Waiting for input")

                """ 
                print affine calibration matrix numbers to terminal
                """ 
                print self.video.aff_matrix

    def affine_cal(self):
        """ 
        Function called when affine calibration button is called.
        Note it only chnage the flag to record the next mouse clicks
        and updates the status text label 
        """
        self.video.aff_flag = 1 
        self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))
 

    """
    Button 1: Reset Position
    """
    def iSetJointAngle(self, jointIndex, value):
        if (not (jointIndex == 0 or jointIndex == 1 or jointIndex == 2 or jointIndex == 3)):
            print("Error: in iSetJointAngle(self, jointIndex, value): jointIndex should be integer in {0,1,2,3}")
            pass
        elif (not (value <= PI and value >= - PI)):
            print("Error: in iSetJointAngle(self, jointIndex, value): value should be from - PI to PI")
            pass
        else:
            self.rex.joint_angles[jointIndex] = value
            if (jointIndex == 0):
                self.ui.sldrBase.setProperty("value",0)
            elif (jointIndex == 1):
                self.ui.sldrShoulder.setProperty("value",0)
            elif (jointIndex == 2):
                self.ui.sldrElbow.setProperty("value",0)
            elif (jointIndex == 3):
                self.ui.sldrWrist.setProperty("value",0)
            else:
                print("iSetJointAngle(self,joitnIndex,value): Unexpected jointIndex value.")
                pass
    """
    Button 2: Reset Torque and Speed
    """
    def iSetTorque(self, value):
        if (not(value >= 0 and value <= 1)):
            print("ERROR: In iSetTorque(self, value): value should be in range [0,1]");
            pass
        else:
            self.rex.max_torque = value;
            self.ui.rdoutTorq.setText(str(100 * value) + "%")
            self.ui.sldrMaxTorque.setProperty("value",value*100)        

    def iSetSpeed(self, value):
        if (not(value >= 0 and value <= 1)):
            print("ERROR: In iSetSpeed(self, value): value should be in range [0,1]");
            pass
        else:
            self.rex.speed = value;
            self.ui.rdoutSpeed.setText(str(100 * value) + "%")
            self.ui.sldrSpeed.setProperty("value",value*100)        

             
    def iResetPosition(self):
        self.iSetJointAngle(0,0)
        self.iSetJointAngle(1,0)
        self.iSetJointAngle(2,0)
        self.iSetJointAngle(3,0)
        self.rex.cmd_publish()
    def iResetTorqueAndSpeed(self):
        self.iSetTorque(0.0)
        self.iSetSpeed(0.1)
        self.rex.cmd_publish()


    """
    Button 3: Start Train.
    """
    def iTrain_ClearRecord(self):
        self.rex.wpt = []
        self.rex.wpt_number = 0
        self.rex.wpt_total = 0
        self.rex.way = []
        self.rex.way_number = 0
        self.rex.way_total = 0
        print("hello.")

    def iTrainBegin(self):
        self.iTrain_ClearRecord()
        self.rex.plan_status = 2
        



    """
    In Play, keep recording.
    """
    def iTrain_FetchSensorData(self):
        return [self.rex.joint_angles_fb[0],
                self.rex.joint_angles_fb[1],
                self.rex.joint_angles_fb[2],
                self.rex.joint_angles_fb[3] ]

    def iTrain_AddOneWay(self):
        SensorData = self.iTrain_FetchSensorData()
        self.rex.way.append(SensorData)
        self.rex.way_total = self.rex.way_total + 1 # Have such data point up to now.
        print("[#way="),
        print(self.rex.way_total),
        print(",#wpt="),
        print(self.rex.wpt_total),
        print("]");


    """
    Get Way Point:
    """
    def iGetWayPoint(self):
        SensorData = self.iTrain_FetchSensorData()
        self.rex.wpt.append(SensorData)
        self.rex.wpt_total = self.rex.wpt_total + 1 # Have such data point up to now.
        
        print("[#way="),
        print(self.rex.way_total),
        print(",#wpt="),
        print(self.rex.wpt_total),
        print("]");
        

    """
    Train stop:
    """
    def iTrainStop(self):
        self.rex.plan_status = 0
        print("Stop Recording!");


    """
    Replay
    """
    def iReplayBegin(self):
        self.rex.plan_status = 5
        self.rex.way_number = 0
        print("Replay Start")

    """
    """
    def iReplayStop(self):
        self.rex.plan_status = 0;
        self.rex.way_number = 0;

    def iReplay_SetOneSensorData(self,valueIndex):
        sensorData = self.rex.way[valueIndex]
        self.iSetJointAngle(0,sensorData[0])
        self.iSetJointAngle(1,sensorData[1])
        self.iSetJointAngle(2,sensorData[2])
        self.iSetJointAngle(3,sensorData[3])
        self.rex.cmd_publish();

    def iReplay_PlayOneWay(self):
        if (self.rex.way_number == self.rex.way_total):
            self.iReplayStop()
        else:
            self.iReplay_SetOneSensorData(self.rex.way_number)
            self.rex.way_number = self.rex.way_number + 1
            print("Playing:["),
            print(self.rex.way_number),
            print(","),
            print(self.rex.way_total),
            print("]");

    """
    Button Avalibity
    """
    def iSetButtonAbility(self):
        if (self.rex.plan_status == 0):
            self.ui.btnUser4.setEnabled(True)
            self.ui.btnUser5.setEnabled(False)
            self.ui.btnUser6.setEnabled(False)
        if (self.rex.plan_status == 2):
            self.ui.btnUser4.setEnabled(False)
            self.ui.btnUser5.setEnabled(True)
            self.ui.btnUser6.setEnabled(True)



def main():
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    ex.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()

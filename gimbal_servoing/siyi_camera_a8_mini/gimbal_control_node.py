from gimbal_viservo.msg import GimbalState
from gimbal_viservo.msg import SpeedControl
from gimbal_viservo.srv import ModeSwitch,ModeSwitchRequest,ModeSwitchResponse

# import time
import rospy
import os
import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from siyi_sdk import SIYISDK

class MODE:
    SPEED_MODE = 1
    ANGLE_MODE = 2
    RETURN_MODE = 3


def sentSpeed(data):
    global Mode,expAngSpeed
    expAngSpeed[0] = data.yaw_speed
    expAngSpeed[1] = data.pitch_speed
   

def sentAngle(expYaw,expPitch):
    global expAngle,expAngSpeed
    expAngle[0] = expYaw
    expAngle[1] = expPitch

    expAngSpeed[0] = 0
    expAngSpeed[1] = 0
    if cam.requestGimbalAngle(expAngle[0],expAngle[1])== True:
        rospy.loginfo("Going to the expected angle!")
        return True
        

def returnHome():

    flag_return = cam.requestCenterGimbal()

    if flag_return == True:
        cam.requestGimbalSpeed(0,0)
        rospy.loginfo("Returned to the home postion!")
        return True


def modeSwith(req):
    global Mode
    Mode = req.mode
    if Mode == MODE.SPEED_MODE:
        return ModeSwitchResponse(True,"Switched to speed control mode!")
    
    elif Mode == MODE.ANGLE_MODE:
        if sentAngle(req.exp_yaw,req.exp_pitch) == True:
            return ModeSwitchResponse(True,"Expected angle reached!")
    
    elif Mode == MODE.RETURN_MODE:
        if returnHome() == True:
            return ModeSwitchResponse(True,"Returned to the home postion!")
    

def run():
    rospy.init_node('gimbal_control_node',anonymous=True)

    state_pub = rospy.Publisher('/gimbal_control_node/gimbal_state',GimbalState,queue_size=10)

    rospy.Subscriber("/gimbal_viservo_controller_node/reqSpeed",SpeedControl,sentSpeed)

    contrServ = rospy.Service('mode_switch_server',ModeSwitch,modeSwith)

    cam.requestHardwareID()

    rate = rospy.Rate(30)
    
    # rospy.spin()


    while not rospy.is_shutdown():

        State = GimbalState()
        angleRPY = cam.getAttitude()
        angularSpeedRPY = cam.getAttitudeSpeed()

        if Mode == MODE.SPEED_MODE:
            d_kp = 2
            newYawSpeed = int(expAngSpeed[0]+d_kp*(expAngSpeed[0]+angularSpeedRPY[0]))
            newPitchSpeed = int(expAngSpeed[1]+d_kp*(expAngSpeed[1]-angularSpeedRPY[1]))

            cam.requestGimbalSpeed(newYawSpeed,newPitchSpeed)
            rospy.loginfo_throttle(3,"Moving in speed control mode under yaw_speed:%s,pitch_speed:%s"\
                                ,expAngSpeed[0],expAngSpeed[1])


        State.exp_yaw = expAngle[0]/10
        State.exp_pitch = expAngle[1]/10
        State.exp_yaw_speed = expAngSpeed[0]
        State.exp_pitch_speed = expAngSpeed[1]

        State.current_yaw = angleRPY[0] #Current yaw
        State.current_pitch = angleRPY[1] #Current pitch
        State.current_roll = angleRPY[2] #Current roll
        State.current_yaw_speed = -angularSpeedRPY[0] #Current yaw_speed
        State.current_pitch_speed = angularSpeedRPY[1] #Current pitch_speed
        State.current_roll_speed = angularSpeedRPY[2] #Current roll_speed
        
        State.header.frame_id = "gimbalState"
        State.header.stamp = rospy.Time.now()
        state_pub.publish(State)

        rate.sleep()
        
    cam.disconnect()


if __name__ == "__main__":

    expAngle = [0,-900]
    expAngSpeed = [0,0]

    try:
        Mode = MODE.SPEED_MODE #Default speed control mode
        cam = SIYISDK(server_ip="192.168.144.25", port=37260)
        if not cam.connect():
            rospy.loginfo("Gimbal connection error!")
            exit(1)
        
        #camera orientation initialization
        if cam.requestGimbalAngle(expAngle[0],expAngle[1]) == True:
            rospy.loginfo("The camera is pointing straight down!")
            run()
    except rospy.ROSInterruptException:
        pass
    
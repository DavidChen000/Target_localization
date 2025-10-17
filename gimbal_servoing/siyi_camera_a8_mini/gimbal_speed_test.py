from gimbal_viservo.msg import SpeedControl

# import time
import rospy
import math


class sineWavePublisher:
    def __init__(self) :

        rospy.init_node('gimbal_speed_test_node',anonymous=True)
        self.speed_pub = rospy.Publisher('/gimbal_viservo_controller_node/reqSpeed',SpeedControl,queue_size=10)
        
        self.pubRate_ = rospy.get_param('~rate',30)
        self.amplitude = rospy.get_param('~amplitude',30.0)
        self.frequency = rospy.get_param('frequency',0.25)

        self.rate = rospy.Rate(self.pubRate_)
        self.time = 0.0
        self.cycle_count = 0
        self.expSpeed = SpeedControl()
    
    def sentYawSpeed(self,speedValue_):
        
        self.expSpeed.yaw_speed = speedValue_

    def sentPitchSpeed(self,speedValue_):
        
        self.expSpeed.pitch_speed = speedValue_

    
    def run(self):
         
        while not rospy.is_shutdown():
            
            speedValue_ = int(self.amplitude*math.sin(2*math.pi*self.frequency*self.time))

            if(self.cycle_count%2):
                self.sentYawSpeed(speedValue_)
            else:
                self.sentPitchSpeed(speedValue_)

            self.speed_pub.publish(self.expSpeed)
            
            self.time += 1.0 / self.pubRate_

            if self.time >= 4.0 /self.frequency:
                self.cycle_count += 1
                self.time = 0.0

            self.rate.sleep()



if __name__ == "__main__":
    try:
        
        sine_wave_Publisher = sineWavePublisher()
        sine_wave_Publisher.run()
        
    except rospy.ROSInterruptException:
        pass
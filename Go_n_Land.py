#!/usr/bin/env python3

#---- Robotic Systems------#
import rospy
import tf
import numpy as np
from crazyflie_driver.msg import Hover
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
#------------------------------------------------------------------------------------------#

class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix
        self.x=0   # Drone X Position
        self.y=0   # Drone Y Position
        self.z=0   # Drone Z Position
        self.x_d=0   # Base X Position
        self.y_d=0   # Base Y Position
        self.x_goal=0   # Drone Goal X Position
        self.y_goal=0   # Drone Goal Y Position

        # Set World Frame
        worldFrame = rospy.get_param("~worldFrame", "/world")

        # Set Rate
        self.rate = rospy.Rate(10)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        # Define Subscriber for Drone Position (Topic: cf1\log1) 
        self.sub=rospy.Subscriber(prefix +'/log1',GenericLogData,self.callback)

        # Define Subscriber for Base Position (Topic: cf2\log1) 
        self.sub_2 = rospy.Subscriber('/cf2/log1',GenericLogData,self.callback_2)

        self.pub = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.msg = Hover()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

    # Get Drone Coordinates
    def callback(self,msg):
        self.x=msg.values[0]
        self.y=msg.values[1]
        self.z=msg.values[2]
        print(self.prefix , ":" , round(self.x,2), "\t", round(self.y,2), "\t", round(self.z,2))

    # Get Base Coordinates
    def callback_2(self,msg):
        self.x_d=msg.values[0]
        self.y_d=msg.values[1]
        print("cf2" , ":" , round(self.x_d,2), "\t", round(self.y_d,2), "\t")
        
    # Update Parameters
    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])


    def Go_n_Land(self):
        start = rospy.get_time()
        
        # Define Staring PI params
        Kp=1
        Ki=0.5
        
        rostime_now = rospy.get_rostime()
        time_now = rostime_now.secs + (rostime_now.nsecs) / 1e9
        
        e_integral_x=0
        e_integral_y=0
        counter=0
        
        zDistance=1.2

        while not rospy.is_shutdown():
            i=False
            while True:
                # The lower is the drone the tighter are the bounds
                bounds = zDistance/6

                # Get the Goal Coords
                xd = self.x_goal
                yd = self.y_goal

                # If the drone is within bounds then descent
                if (xd - bounds) <= self.x <= (xd + bounds) and (yd - bounds) <= self.y <= (yd + bounds) :
                    now_1=rospy.get_time()
                    
                    print("Descent")
                    zDistance-=zDistance/47.5 # Lower Z

                    # If Lower than 0.5 meters then Pi is more aggressive
                    if(zDistance>=0.5):
                        Kp=1
                        Ki=0.5
                        
                    if(zDistance<0.5):
                        Kp=1.2
                        Ki=0.8

                print("Go to:",xd,yd)

                # Calc erros
                errorx=xd-self.x
                errory=yd-self.y
                errorz=zDistance-self.z
                
                # P implementation
                ux = Kp * errorx 
                uy = Kp * errory
                uz = Kp * errorz
                
                # I implementation
                time_prev = time_now
                rostime_now = rospy.get_rostime()
                time_now = rostime_now.secs + (rostime_now.nsecs) / 1e9
                delta_time = time_now - time_prev
                
                e_integral_x+= errorx*delta_time
                e_integral_y+= errory*delta_time

                ux += Ki * e_integral_x
                uy += Ki * e_integral_y

                # Send new Pos
                self.msg.vx = ux
                self.msg.vy = uy
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance

                now = rospy.get_time()

                # If to much time has passed the break
                if (now - start > 40):
                   print("Time Out...")
                   break

                # If lower than 0.27 meters then Shut off and Land
                if (zDistance <0.27 ):
                    counter+=1
                    if(counter==1):    
                        print("Landing...")
                        self.stop_pub.publish(self.stop_msg)
                        break

                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break



    
    # Drone Take Off up to Z Height
    def takeOff(self, zDistance):
        # Step
        time_range = 1 + int(10*zDistance/0.6)
        
        while not rospy.is_shutdown():
            for i in range(time_range):
                self.msg.vx = 0.0 # X stays at 0
                self.msg.vy = 0.0 # Y stays at 0
                self.msg.yawrate = 0.0 # No Yaw
                self.msg.zDistance = i / (time_range/zDistance) # Send new Z for each Step
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
                
            # After Take off, Hover for 20 seconds
            for y in range(20):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg)
                self.rate.sleep()
            break


#  Handler for controlling Drone
def handler(cf):
    cf.takeOff(1.2)
    cf.Go_n_Land()

# Estimate Position of Base
def estimate_base(cf):
    start=rospy.get_time()
    
    while True:
        start=rospy.get_time()
        i=0
        arrx=np.array([])
        arry=np.array([])

        # Get for Positions for 0.3 seconds
        while(True):
            i+=1
            arrx=np.append(arrx,cf.x_d)
            arry=np.append(arry,cf.y_d)        
            now=rospy.get_time()
            if(now-start>0.3):
                break

        # Estimate X coordinate
        meanx=np.mean(arrx)
        stdx=np.std(arrx)
        if(stdx>1):
            threshold=2
            arrx= arrx[np.abs(arrx - meanx) <= threshold * stdx]
        xd=round(arrx.mean(),3)
        
        # Estimate Y coordinate
        meany=np.mean(arry)
        stdy=np.std(arry)
        if(stdy>0.2):
            threshold=2
            arry= arry[np.abs(arry - meany) <= threshold * stdy]
        yd=round(arry.mean(),3)

        # Finally set the estimates as the Goal Parameter
        cf.x_goal=xd
        cf.y_goal=yd
        
        stop=rospy.get_time()
        if(stop-start>40):
            break

# Main
if __name__ == '__main__':
    # Start Node
    rospy.init_node('hover', anonymous=True)

    # Create CrazyFlie Object
    cf1 = Crazyflie("cf1")

    # Estimate Base Position Thread
    t0 = Thread(target = estimate_base, args = (cf1,) )
    t0.start()

    # Controlling Drone Thread
    t1 = Thread(target = handler, args = (cf1,) )
    t1.start()




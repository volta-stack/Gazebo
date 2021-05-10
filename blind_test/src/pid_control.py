#!/use/bin/env python 
import rospy 
from std_msgs.msg import Float64
import time

class PID(object): 
    def __init__(self, init_setPoint_value, init_state):
        self.setPoint_value = init_setPoint_value
        self.state_value = init_state

        self._setpoint_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self._state_pub = rospy.Publisher("/state", Float64, queue_size=1) 
        
        self._control_effort_sub = rospy.Subscriber('/control_effort', Float64, self.control_effort_callback) 
        self._control_effort_value = Float64() 

        self._control_effort_ready()

    def control_effort_callback(self, data):
        try:
            self._control_effort_value.data = data.data 
        except:
            rospy.logerr(
                "Current /control_effort not ready yet, retrying for getting control_effort"
            )       

    def control_effort_ready(self): 
        self._control_effort_value = None 

        rospy.logdebug("Waiting for /control_effort to be READY...")

        while self._control_effort_value is None and not rospy.is_shutdown():
            rospy.logdebug("Publishing Initial State and Setpoint...")
            rospy.logdebug("State="+str(self.state_value)+",SetValue="+str(self.setPoint_value))

            self.state_update(value=self.setPoint_value)
            self.setPoint_update(value=self.setPoint_value)

            rospy.logdebug("Publishing Initial State and Setpoint...DONE")
            try:
                self._control_effort_value = rospy.wait_for_message(
                    '/control_effort', Float64, timeout=1.0)
                rospy.logdebug("Current /control_effort READY=>")

            except:
                rospy.logerr(
                    "Current /control_effort not ready yet, retrying for getting control_effort")
    
    def setpoint_update(self, value):
        self.setPoint_value = value
        value_object = Float64()
        value_object.data = self.setPoint_value
        self._setpoint_pub.publish(value_object)

    def state_update(self, value):
        self.state_value = value
        value_object = Float64()
        value_object.data = self.state_value
        self._state_pub.publish(value_object)

    def get_control_effort(self):
        return self._control_effort_value.data


class VEL_R(object):
    def __init__(self):
        self._vel_r_sub = rospy.Subscriber('/vel_r', Float64, self.vel_r_callback) 
        self._vel_r = Float64() 

        self._vr_sub = rospy.Subscriber('/vr', Float64, self.vr_callback) \
        self._vr = Float64()     

        self._pid_vr_pub = rospy.Publisher("/pid_vr",Float64,queue_size=1)
        
    def vel_r_callback(self, data):
        self.vel_r.data = data.data

    def get_vel_r(self):
        return self.vel_r.data

    def vr_callback(self, data):
        self.vr.data = data.data
    
    def get_vr(self):
        return self.vr.data

    def pid_vr_pub(self, pid_vr):
        self._pid_vr_pub.publish(pid_vr)


def vel_test():
    rospy.init_node('vr_pid_test', anonymous=True)

    vel_r_object = VEL_R()
    
    setPoint_value = vel_r_object.get_vr
    state_value = vel_r_object.get_vel_r

    pid_object = PID(init_setPoint_value= vr
                        ,init_state= vel_r
                    )
    
    rate = rospy.Rate(15.0)
    ctrl_c = False 

    def shutdownhook(): 
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        setPoint_value = vel_r_object.get_vr
        pid_object.setpoint_update(value=setPoint_value)

        state_value = vel_r_object.get_vel_r
        pid_object.state_update(value=state_value)

        effort_value = pid_object.get_control_effort()
        vel_r_object.pid_vr_pub(effort_value)

        rospy.loginfo("setPoint_value ==>"+str(setPoint_value))
        rospy.loginfo("state_value ==>"+str(state_value))
        rospy.loginfo("effort_value ==>"+str(effort_value))

        rate.sleep()

if __name__ == '__main__': 
    vel_test()
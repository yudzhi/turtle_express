#!/usr/bin/env python
#import roslib; roslib.load_manifest('smach_preemption_example')
import rospy
import smach
import smach_ros
from clever import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray, Pose
#Charging
from sensor_msgs.msg import BatteryState
#Camera
from mavros_msgs.msg import camera_message_type


class Base(smach.State):
    """
        INPUT:
            База. Ждёт, пока дрон зарядится.
        OUTPUT: Publisher('drone_state') => 'Ready'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done', 'bat_error'])
    def execute(self, userdata):
        while True:
            rospy.sleep(5)
            try:
                if battery.voltage > 15.8:
                    rospy.loginfo("Low Battery, I need to go home, I'll be back")
                    pub.publish("Low Battery!")
                    return 'setup_done'
                    break
            except:
                rospy.loginfo("Battery Error")
                    return 'bat_error'

class Bat_Monitor(smach.State):
    """
        INPUT:
        OUTPUT: Publisher('drone_state') => 'Low Battery!'
                rospy.Publisher('/sm_reset', Empty)
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['low_bat', 'bat_error'])
    def execute:
        while True:
            time.sleep(1)
            try:
                if battery.voltage < 15.2:
                    rospy.loginfo("Low Battery, I need to go home, I'll be back")
                    pub.publish("Low Battery!")
                    return 'low_bat'
#                    latched - new subscribers that come online after will hear this message
                    pubodom = rospy.Publisher('/sm_charge', Empty, queue_size = 10, latch = True)
                    break
            except:
                rospy.loginfo("Battery Error")
                return 'bat_error'

class Camera_Search(smach.State):
    """
        INPUT: geometry_msgs/PoseArray.msg
            (std_msgs/Header header
            geometry_msgs/Pose[] poses
        Сравнивает полученные координаты с уже имеющимися в логе (обрезает до int)
        Если нет в списке - постит в топик 'drone_state' и дописывает лог
        OUTPUT: Publisher('drone_state') => 'Object Found!"
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_detected'])
    def execute:
        while True:
            time.sleep(1)
            if object_spots:
                pub.publish("Object detected!")
                #latched - new subscribers that come online after will hear this message
                pubodom = rospy.Publisher('/sm_hover', Empty, queue_size = 10, latch = True)
                break


class Flying(smach.State):
    """
        INPUT: trajectory file
        OUTPUT: Publisher('drone_state') => 'Flying mode'
    """
    def __init__(self):
        smach.State.__init__(self, outcomes = ['all_clear', 'preempted'])
    def execute(self, trajectory):
        for pose in trajectory:
            if self.preempt_requested():
                rospy.loginfo("Полёт прерван")
                self.service_preempt()
                return 'preempted'
#                    TODO: 1. flying algoritm
#                        2. Flag to distinguish Battery & Camera alarm
#                            3. trajectory filename into main()
        return 'all_clear'

class Landing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['landing_succeded'])
    def execute(self):
        pass

# gets called when ANY child state terminates
def child_term_cb(outcome_map):
    if outcome_map['PATROL_MODE'] == 'all_clear':
        return True
    # terminate all running states if RETURN_TO_BASE finished
    elif outcome_map['RETURN_TO_BASE']:
        return True
    else:
        # in all other case, just keep running, don't terminate anything
        return False

# gets called when ALL child states are terminated
def out_cb(outcome_map):
    if outcome_map['PATROL_MODE'] == 'all_clear':
        return 'patrol_done'
    else:
        return 'low_bat'

def monitor_cb(ud, msg):
    """
        Arguments are the userdata and the message. This needs to return False
        when we want the monitor state to terminate.
        In this case, the monitor state will return 'invalid'
    """
    return False

def battery_update(data):
    global battery
    battery = data

def HumFound(data):
    global object_spots
    x = int(data.poses.position.x)
    y = int(data.poses.position.y)
    xy_pos = [x, y]
    #    f = open('ObjCoordinates.txt','r')
    #    str_coord = f.readlines()
    #        for sd in str_coord:
    #            history.append([int(sd[0]),int(sd[2])])
    for sd in objects_history:
        if not xy_pos in objects_history:
            objects_history.append(xy_pos)
            object_spots = xy_pos
    #            f = open("Obj Coordinates.txt", 'a')
    #            f.write(str(x) + ' ' + str(y) + '\n')
    #            f.close()

def main():
    rospy.init_node('st_mach')
    rospy.loginfo('Inited node st_mach')
    st_mach_pub = rospy.Publisher('/drone_state', String, queue_size=10)
    rospy.Subscriber('/mavros/battery', BatteryState, battery_update)
    rospy.Subscriber('/detected_object', PoseArray, HumFound)
    
    objects_history = []
    
    # creating the concurrence state machine
    flying_concurrence = smach.Concurrence(outcomes=['patrol_done', 'low_bat'],
                                           default_outcome = 'patrol_done',
                                           child_termination_cb = child_term_cb,
                                           outcome_cd = out_cb)

    with flying_concurrence:
        smach.Concurrence.add('START_HOVERING', smach_ros.MonitorState("/sm_hover", Empty, monitor_cb))
        smach.Concurrence.add('RETURN_TO_BASE', smach_ros.MonitorState("/sm_charge", Empty, monitor_cb))
        smach.Concurrence.add('PATROL_MODE', Flying())
        smach.Concurrence.add('BATTERY_MONITOR', Bat_Monitor())
        smach.Concurrence.add('OBJECT_FOUND', Camera_Search())

    sm = smach.StateMachine(outcomes = ['charging'])
    with sm:
        smach.StateMachine.add('BASE_CHARGING', Base(),
                               transitions = {'setup_done' : 'FLYING', 'bat_error' : 'SHUT_DOWN'})
        smach.StateMachine.add('FLYING', Flying(),
                               transitions = {'patrol_done':'RETURN_TO_BASE', 'low_bat':'RETURN_TO_BASE'})
        smach.StateMachine.add('RETURN_TO_BASE', Flying(),
                               transitions = {'over_base':'LANDING'})
        smach.StateMachine.add('LANDING', Landing(),
                               transitions = {'landing_succeded':'charging'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()


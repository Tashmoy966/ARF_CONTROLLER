#!/usr/bin/env python  
import numpy as np
import rospy
import argparse
from math import sqrt ,pow, atan2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from potential_field_planning import ARF

Robot_Names=["robot1","robot2","robot3","robot4"]
distance_tolernce=0.2
class GroundTruth(object):
    def __init__(self,name):
        #print(name)
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.callback,queue_size=10)
        self._pose=np.array([np.nan,np.nan,np.nan],dtype=np.float32)
        self._name=name
    def callback(self,msg):
        idx=[i for i,n in enumerate(msg.name) if n==self._name]
        if not idx:
            raise ValueError('Name: "{}" not exist'.format(self._name))
        idx=idx[0]
        self._pose[0]= msg.pose[idx].position.x
        self._pose[1]= msg.pose[idx].position.y
        _,_,yaw=euler_from_quaternion([msg.pose[idx].orientation.x,msg.pose[idx].orientation.y,msg.pose[idx].orientation.z,msg.pose[idx].orientation.w])
        self._pose[2]=yaw
        # print(self._pose)
    @property
    def ready(self): 
        return not np.isnan(self._pose[0])
    @property
    def r_pose(self):
        # print(self._pose)
        return self._pose
def euclidian_distance(next_pose,curr_pose):
    #print(curr_pose)
    return sqrt(pow((next_pose[0]-curr_pose[0]),2) + pow((next_pose[1]-curr_pose[1]),2))

def steering_angle(next_pose,curr_pose):
    return atan2((next_pose[1]-curr_pose[1]),(next_pose[0]-curr_pose[0]))

def run(gc):
    rospy.init_node('apf', anonymous=True)

    rate=rospy.Rate(50)

    publishers=[None]*len(Robot_Names)
    ground_truths=[None]*len(Robot_Names)
    vel_msg=[None]*len(Robot_Names)

    for i,name in enumerate(Robot_Names):
        publishers[i]=rospy.Publisher('/'+name+'/cmd_vel',Twist,queue_size=10)
        ground_truths[i]=GroundTruth(name)
        # print(ground_truths[i].pose)
    while not rospy.is_shutdown():
        #rospy.wait_for_service('/gazebo/model_states')
        if not all (groundtruth.ready for groundtruth in ground_truths):
            rate.sleep()
            continue
        robot_poses=np.array([ground_truths[i].r_pose for i in range(len(ground_truths))])
        #print(robot_poses)
        goal_pos=np.array([[gc[0]+2,gc[1]+2,-1.5708],[gc[0]+2,gc[1]-2,-1.5708],[gc[0]-2,gc[1]-2,-1.5708],[gc[0]-2,gc[1]+2,-1.5708]])
        #print(goal_pos)
        arf=ARF(robot_poses[:,0:2],goal_pos)
        next_poses=arf.potential()   #robot_poses,goal_pos
        
        #print(next_poses)
        robot_poses=np.array([ground_truths[i].r_pose for i in range(len(ground_truths))])
        for r in range(len(vel_msg)):
            # print("#########")
            # print(robot_poses)
            ed=euclidian_distance(next_poses[r],robot_poses[r])
            vel_msg[r]=Twist()
            vel_msg[r].linear.x=0.1*ed
            vel_msg[r].linear.y=0
            vel_msg[r].linear.z=0
            #print(1.5*ed)
            # rate.sleep()
            vel_msg[r].angular.x=0
            vel_msg[r].angular.y=0
            vel_msg[r].angular.z=0.1*(steering_angle(next_poses[r],robot_poses[r])-robot_poses[r][2])
            print(vel_msg[r].angular.z)
            if euclidian_distance(robot_poses[r],goal_pos[r])<=distance_tolernce:
                vel_msg[r].linear.y=0
                vel_msg[r].linear.z=0
                vel_msg[r].linear.x=0

                vel_msg[r].angular.x=0
                vel_msg[r].angular.y=0
                vel_msg[r].angular.z=0    #1.5*(steering_angle(next_poses[r],robot_poses[r])-robot_poses[r][2])
        for r in range(len(Robot_Names)):
            publishers[r].publish(vel_msg[r])
            rate.sleep()
        del arf
        


if __name__=="__main__":
    parser= argparse.ArgumentParser(description="Goal Center")
    parser.add_argument('--goal_center', action='store',default="[10.0,10.0]",type = float,nargs="*",help='Common Center For All Robot in Goal Position')
    args,unknown=parser.parse_known_args()
    try:
        run(args.goal_center)
        # rospy.spin()
    # If we press control + C, the node will stop.
    except rospy.ROSInternalException:
        pass
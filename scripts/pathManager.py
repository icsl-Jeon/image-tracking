#!/usr/bin/env python
import rospy
import nav_msgs 
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class pathManager:
    def __init__(self,target_name,observation_rate,observation_stack_size,pred_time,pred_num):
        self.target_name=target_name
        self.observation_stack_size=observation_stack_size
        
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        self.pub = rospy.Publisher("target_predition_path",nav_msgs.msg.Path)
        
        self.obs_stack =[] # list of Point of target
        self.time_stack =[]
        self.predict_stack=Path() # prediction stack (list of PoseStamped=path ) 
        self.predict_stack.header.frame_id="world"
        self.pred_time = pred_time  # how long predict
        self.pred_num = pred_num  # how many predict
        self.obsservation_rate=observation_rate
        
    def gazebo_callback(self,msg):

        name_list=list(msg.name)
        model_idx=name_list.index(self.target_name)
        target_pose=msg.pose[model_idx]
        now=rospy.get_time()
        if len(self.obs_stack) == self.observation_stack_size: 
            if now-self.time_stack[-1] >= self.obsservation_rate :
                self.time_stack.pop(0)
                self.obs_stack.pop(0) # discharge the old value
                self.obs_stack.append(target_pose.position)
                self.time_stack.append(rospy.get_time())
        else :
            if len(self.time_stack)==0  :
                self.obs_stack.append(target_pose.position)
                self.time_stack.append(rospy.get_time())
            else:
                if now-self.time_stack[-1] >= self.obsservation_rate:
                    self.obs_stack.append(target_pose.position)
                    self.time_stack.append(rospy.get_time())

            
    def target_prediction(self):
        now=rospy.get_time() ## reference time

        if len(self.obs_stack):

            ts=np.array(self.time_stack)-self.time_stack[-1]
            obs_span=ts[-1]-ts[0]
            ts=ts/obs_span+1  # normalize to [0,1]
            #print ts
            # normalize for stablity
            xs=[]; ys=[]; zs=[]
        # just perform regression as much as the stack has
            for i in range(min(self.observation_stack_size,len(self.obs_stack))):
                xs.append(np.array(self.obs_stack[i].x))
                ys.append(np.array(self.obs_stack[i].y))
                zs.append(np.array(self.obs_stack[i].z))

            # constant acceleration model
            px = np.polyfit(ts, xs, 1)
            py = np.polyfit(ts, ys, 1)
            pz = np.polyfit(ts, zs, 1)

            pred_path=Path()
            pred_path.header.frame_id="world"
            pred_path.poses=[]
            # prediction stack
            for t_eval in np.linspace(0,self.pred_time,self.pred_num):
                t_eval=t_eval/obs_span+1
                msg=PoseStamped()
                msg.pose.position.x=np.polyval(px, t_eval)
                msg.pose.position.y=np.polyval(py, t_eval)
                msg.pose.position.z=np.polyval(pz, t_eval)
                pred_path.poses.append(msg)

            self.predict_stack=pred_path

        else:

            rospy.logwarn("observation stack is empty")

        
        
    def publish(self):
        if self.predict_stack:
            self.pub.publish(self.predict_stack)
        else:
            rospy.logwarn("not prediction path to publish yet")
        
        

if __name__ == "__main__":
    
    rospy.init_node('path_manager')
    target_name="target"
    observation_stack_size=5
    pred_time=3 # prediction horizon
    pred_num=5 # not that important ...

    manager=pathManager(target_name,0.05,observation_stack_size,pred_time,pred_num)
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        manager.target_prediction()

        manager.publish()
        r.sleep()







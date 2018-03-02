#!/usr/bin/env python
import rospy
import nav_msgs 
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class pathManager:
    def __init__(self,target_name,observation_stack_size,pred_time,pred_num):
        self.target_name=target_name
        self.observation_stack_size=observation_stack_size
        
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        self.pub = rospy.Publisher("target_predition_path",nav_msgs.msg.Path,10)
        
        self.obs_stack =[] # list of Pose of target
        self.time_stack =[]
        self.predict_stack=[] # prediction stack (list of PoseStamped=path ) 
        self.pred_time=pred_time # how long predict 
        self.pred_num=pred_num # how many predict 
        
        
    def gazebo_callback(self,gazebo_msg):
        name_list=list(gazebo_msg.name)
        model_idx=name_list.index(self.target_name)
        target_pose=gazebo_msgs.pose[model_idx]
        if len(self.obs_stack) == self.observation_stack_size: 
            self.time_stack.pop(0)
            self.obs_stack.pop(0) # discharge the old value
            self.obs_stack.append(obs_target_pose.position)
            self.time_stack.append(rospy.get_time())
        else :
            self.obs_stack.append(obs_target_pose.position)
            self.time_stack.append(rospy.get_time())
            
            
    def target_prediction(self):
        now=rospy.get_time() ## reference time 
            
        ts=np.array(self.time_stack)-now
        xs=[]; ys=[]; zs=[]
        if len(self.obs_stack):
        # just perform regression as much as the stack has
            for i in range(min(self.observation_stack_size,len(self.obs_stack))):
                xs.append(np.array(self.obs_stack[i].position.x))
                ys.append(np.array(self.obs_stack[i].position.y))
                zs.append(np.array(self.obs_stack[i].position.z))

            # constant acceleration model
            px = np.polyfit(ts, xs, 2)
            py = np.polyfit(ts, ys, 2)
            pz = np.polyfit(ts, zs, 2)

            pred_path=Path()
            pred_path.poses=[]
            # prediction stack
            for t_eval in np.linspace(0,self.pred_time,self.pred_num):
                msg=PoseStamped()
                msg.pose.position.x=np.polyval(px, t_eval)
                msg.pose.position.y=np.polyval(py, t_eval)
                msg.pose.position.z=np.polyval(pz, t_eval)
                path.poses.append(msg)

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
    observation_stack_size=10
    pred_time=1
    pred_num=5
    manager=pathManager(target_name,observation_stack_size,pred_time,pred_num)
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        manager.target_prediction()
        manager.publish()
        r.sleep()
       





# In[36]:





#!/usr/bin/env python
# realease version
import rospy
import nav_msgs
import numpy as np
import itertools
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

from tf import transformations
from cvxopt import matrix, solvers
from image_tracking.msg import ProposalBox
from image_tracking.msg import ProposalBoxes
from image_tracking.msg import ProposalRays
from image_tracking.msg import ProposalRay
from image_tracking.msg import ProposalBoxPath

class pathManager:
    def __init__(self, target_name, observation_rate, observation_stack_size, pred_time, pred_num,tracking_distance):
        self.target_name = target_name
        self.observation_stack_size = observation_stack_size

        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        self.sub_PB=rospy.Subscriber("/proposal_box_path",ProposalBoxPath,self.PBs_callback)
        self.pub = rospy.Publisher("target_predition_path", nav_msgs.msg.Path)
        self.pub_PR = rospy.Publisher("/proposal_ray_path", ProposalRays)
        self.pub_waypoint=rospy.Publisher("/firefly/command/trajectory",MultiDOFJointTrajectory)

        self.obs_stack = []  # list of Point of target
        self.time_stack = []
        self.predict_stack = Path()  # prediction stack (list of PoseStamped=path )
        self.predict_stack.header.frame_id = "world"
        self.predict_start_time=0 # default
        self.pred_time = pred_time  # how long predict
        self.pred_num = pred_num  # how many predict
        self.obsservation_rate = observation_rate
        self.UAV_pose = Pose()
        self.PR_Path = ProposalRays()
        self.tracking_distance=tracking_distance # actual tracking. not just virtual ray


    # 0th axis = azim
    # 1th axis = elev
    def get_PR_Path_matrix(self):
        azim_Path=[]
        elev_Path=[]
        for ray in list(self.PR_Path.Ray_Path):
            azim_Path.append(ray.azimuth)
            elev_Path.append(ray.elevation)

        return np.stack((np.array(azim_Path),np.array(elev_Path)))

    # 0th axis = xs
    # 1th axis = ys
    # 2th axis = zs
    def get_position_Path_matrix(self):
        x_Path=[]
        y_Path=[]
        z_Path=[]

        for pose in list(self.predict_stack.poses):
            x_Path.append(pose.pose.position.x)
            y_Path.append(pose.pose.position.y)
            z_Path.append(pose.pose.position.z)

        return np.array([x_Path,y_Path,z_Path])


    def get_waypoint_from_position_ray(self,x,y,z,azim,elev):
        return np.array([x+self.tracking_distance*np.cos(elev)*np.cos(azim),y+self.tracking_distance*np.cos(elev)*np.sin(azim),z+self.tracking_distance*np.sin(elev)])

    def gazebo_callback(self, msg):

        name_list = list(msg.name)
        UAV_idx = name_list.index("firefly")
        self.UAV_pose = msg.pose[UAV_idx]

        model_idx = name_list.index(self.target_name)
        target_pose = msg.pose[model_idx]
        now = rospy.get_time()
        if len(self.obs_stack) == self.observation_stack_size:
            if now - self.time_stack[-1] >= self.obsservation_rate:
                self.time_stack.pop(0)
                self.obs_stack.pop(0)  # discharge the old value
                self.obs_stack.append(target_pose.position)
                self.time_stack.append(rospy.get_time())
        else:
            if len(self.time_stack) == 0:
                self.obs_stack.append(target_pose.position)
                self.time_stack.append(rospy.get_time())
            else:
                if now - self.time_stack[-1] >= self.obsservation_rate:
                    self.obs_stack.append(target_pose.position)
                    self.time_stack.append(rospy.get_time())

    # proposal box callback
    def PBs_callback(self, msg):
        # current azimuth and elevation?

        UAV_position = self.UAV_pose.position
        target_position = self.obs_stack[-1]

        d = np.sqrt(np.power(UAV_position.x - target_position.x, 2) +
                    np.power(UAV_position.y - target_position.y, 2) +
                    np.power(UAV_position.z - target_position.z, 2))
        h = UAV_position.z - target_position.z  # should be positive
        if h < 0:
            print "fuck,"
        h0 = np.arcsin(h / d)
        A0 = np.arctan2(UAV_position.y - target_position.y, UAV_position.x - target_position.x)
        if A0<0:
            A0=A0+2*np.pi
        # predicted and proposed aimuth and elevation


        t_step = len(msg.PB_path) + 1  # 1(current)+5(prediction)
        # optimzation variables
        # [A0 A1 A2 ..An h0 h1 ... hn]
        Q_d_tmp = np.zeros((t_step, t_step))

        # equality constraint : initial azimuth and elevation
        Aeq = np.zeros((2, 2 * t_step))
        Aeq[0][0] = 1
        Aeq[1][t_step] = 1
        beq = np.ones((2, 1))
        beq[0] = A0
        beq[1] = h0

        # variation cost

        Q_d_tmp[0][0] = 1
        Q_d_tmp[-1][-1] = 1
        for idx in range(t_step - 2):
            Q_d_tmp[idx + 1][idx + 1] = 2

        for idx in range(t_step - 1):
            Q_d_tmp[idx][idx + 1] = -1
            Q_d_tmp[idx + 1][idx] = -1

        # Variational cost
        Q_d = np.zeros((2 * t_step, 2 * t_step))

        Q_d[np.ix_(range(0, t_step), range(0, t_step))] = Q_d_tmp
        Q_d[np.ix_(range(t_step, 2 * t_step), range(t_step, 2 * t_step))] = Q_d_tmp

        Q_d = matrix(Q_d)
        q_d = matrix(np.zeros((2 * t_step, 1)))


        # visibility cost  & visiblity constraint

        list_ = []
        for Boxes_iter in msg.PB_path:
            cur_boxes = Boxes_iter.PBs
            list_.append(cur_boxes)

        candidate_BoxPath = list(itertools.product(*list_))


        w_v = 0.3  # variational cost vs visiblity
        cost_list = []
        sol_list = []
        for cur_PB_Path in candidate_BoxPath:
            Q_v = np.zeros((2 * t_step, 2 * t_step))
            q_v = np.zeros((2 * t_step, 1))
            A = np.zeros((4 * (t_step - 1) + 2, 2 * (t_step)))
            b = np.zeros((4 * (t_step - 1) + 2, 1))
            c_v = 0  # should be considered !!
            for idx in range(1, t_step):
                cur_Box = cur_PB_Path[idx - 1]
                A_ic = cur_Box.center.x
                h_ic = cur_Box.center.y
                w_v_Ai = w_v / (cur_Box.upper_right_point.x - cur_Box.lower_left_point.x)*0.5
                w_v_hi = w_v / (cur_Box.upper_right_point.y - cur_Box.lower_left_point.y)
                Q_v[idx][idx] = w_v_Ai
                Q_v[idx + t_step][idx + t_step] = w_v_hi
                q_v[idx] = -2 * A_ic * w_v_Ai
                q_v[idx + t_step] = -2 * h_ic * w_v_hi
                A[2 * idx - 1][idx] = 1
                A[2 * idx][idx] = -1
                A[2 * idx - 1 + 2 * (t_step - 1) + 1][idx + t_step] = 1
                A[2 * idx + 2 * (t_step - 1) + 1][idx + t_step] = -1

                b[2 * idx - 1] = cur_Box.upper_right_point.x
                b[2 * idx] = -cur_Box.lower_left_point.x
                b[2 * idx - 1 + 2 * (t_step - 1) + 1] = cur_Box.upper_right_point.y
                b[2 * idx + 2 * (t_step - 1) + 1] = -cur_Box.lower_left_point.y

                c_v = c_v + w_v_Ai * np.power(A_ic, 2) + w_v_hi * np.power(h_ic, 2)

            A = np.delete(A, 0, 0)
            A = np.delete(A, 2 * (t_step - 1), 0)

            b = np.delete(b, 0, 0)
            b = np.delete(b, 2 * (t_step - 1), 0)

            # np.delete(A,[0 2*(t_step-1)+1],None)

            A = matrix(A)
            b = matrix(b)
            Q_v = matrix(Q_v)
            q_v = matrix(q_v)
            sol = solvers.qp(P=2 * Q_d + 2 * Q_v, q=q_d + q_v, G=A, h=b, A=matrix(Aeq), b=matrix(beq))
            cost_list.append(sol['primal objective'] + c_v)
            sol_list.append(sol['x'])

        PR_sol=sol_list[np.argmin(cost_list)]
        PR_Path = ProposalRays()
        for idx in range(len(PR_sol) / 2 - 1):
            PR = ProposalRay()
            PR.azimuth = PR_sol[idx + 1]
            PR.elevation = PR_sol[idx + 1 + t_step]
            PR_Path.Ray_Path.append(PR)

        self.PR_Path = PR_Path


    def target_prediction(self):
        now = rospy.get_time()  ## reference time

        if len(self.obs_stack):

            ts = np.array(self.time_stack) - self.time_stack[-1]
            obs_span = ts[-1] - ts[0]
            ts = ts / obs_span   # normalize to [0,1]
            # print ts
            # normalize for stablity
            xs = []
            ys = []
            zs = []
            # just perform regression as much as the stack has
            for i in range(min(self.observation_stack_size, len(self.obs_stack))):
                xs.append(np.array(self.obs_stack[i].x))
                ys.append(np.array(self.obs_stack[i].y))
                zs.append(np.array(self.obs_stack[i].z))

            # constant acceleration model
            px = np.polyfit(ts, xs, 1)
            py = np.polyfit(ts, ys, 1)
            pz = np.polyfit(ts, zs, 1)


            self.predict_start_time=rospy.get_time()
            pred_path = Path()
            pred_path.header.frame_id = "world"
            pred_path.poses = []
            # prediction stack
            for t_eval in np.linspace(0, self.pred_time, self.pred_num):
                t_eval = t_eval / obs_span
                msg = PoseStamped()
                msg.pose.position.x = np.polyval(px, t_eval)
                msg.pose.position.y = np.polyval(py, t_eval)
                msg.pose.position.z = np.polyval(pz, t_eval)
                pred_path.poses.append(msg)

            self.predict_stack = pred_path

        else:

            rospy.logwarn("observation stack is empty")

    def pred_publish(self):
        if self.predict_stack:
            self.pub.publish(self.predict_stack)
        else:
            rospy.logwarn("not prediction path to publish yet")


    def waypoint_publish(self):
        t_eval=rospy.get_time()-self.predict_start_time
        ts_sampled=np.linspace(0,self.pred_time,self.pred_num)
        ray_sampled=self.get_PR_Path_matrix()
        azim_sampled=ray_sampled[0]
        elev_sampled=ray_sampled[1]

        position_sampled=self.get_position_Path_matrix()
        if not (azim_sampled)==None:
            if (len(position_sampled[0]) == self.pred_num ) &(len(azim_sampled) ==self.pred_num ):
                interp_y=np.interp(t_eval,ts_sampled,position_sampled[1])
                interp_z=np.interp(t_eval,ts_sampled,position_sampled[2])
                interp_x=np.interp(t_eval,ts_sampled,position_sampled[0])

                interp_azim = np.interp(t_eval, ts_sampled, azim_sampled)
                interp_elev = np.interp(t_eval, ts_sampled, elev_sampled)

                desired_position=self.get_waypoint_from_position_ray(interp_x,interp_y,interp_z,interp_azim,interp_elev)
                desired_yaw=interp_azim+np.pi


                desired_pose = Transform()
                desired_pose.translation.x=desired_position[0]
                desired_pose.translation.y=desired_position[1]
                desired_pose.translation.z=desired_position[2]

                desired_pose.rotation=Quaternion(*transformations.quaternion_about_axis(desired_yaw,[0,0,1]))

                traj_point=MultiDOFJointTrajectoryPoint()
                traj_point.transforms.append(desired_pose)

                traj_msg=MultiDOFJointTrajectory()
                traj_msg.points.append(traj_point)


                self.pub_waypoint.publish(traj_msg)


    def PR_publish(self):
        if self.PR_Path:
            self.pub_PR.publish(self.PR_Path)

        else:
            rospy.logwarn("not proposal ray path to publish yet")

if __name__ == "__main__":

    rospy.init_node('path_manager')
    target_name = "target"
    observation_stack_size = 5
    pred_time = 3  # prediction horizon
    pred_num = 5  # not that important ...
    tracking_distance=3
    manager = pathManager(target_name, 0.05, observation_stack_size, pred_time, pred_num,tracking_distance)

    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        manager.target_prediction()

        manager.pred_publish()
        manager.PR_publish()
        manager.waypoint_publish()
        r.sleep()

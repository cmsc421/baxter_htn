#!/usr/bin/env python
import rospy
from baxter_pickup_msgs.msg import (
    BlockArray,
    Plan,
    Step,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    String,
    Header,
    Empty,
)
import baxter_interface
from gazebo_msgs.srv import GetModelState
from baxter_pickup_msgs.srv import BaxterIK, BaxterIKRequest
import copy
import math

class BaxterPickup:
    def __init__(self, limb, name, ID):
        # Initialize Node
        rospy.init_node('grabber', anonymous=True)
        # Subscribe to sensor data
        #rospy.Subscriber("/block_location", BlockArray, self.model_callback)
        rospy.Subscriber("/plan", Plan, self.plan_callback)
        self._name = name
        self._ID = ID
        # Contains a list of block locations for blocks visible by Baxter
        self._block_locations = []
        self._plan = []
        self._check_for_blocks = False 
        self.respL = []
        self.respR= []
        self.outL = []
        self.outR= []
        # Select limb (left or right)
        self._limb_name = limb
        self._limbL = baxter_interface.Limb("left")
        self._limbR = baxter_interface.Limb("right")
        # Set movement speed
        self._limbL.set_joint_position_speed(0.05)
        self._limbR.set_joint_position_speed(0.05)
        # Initialize Gripper
        self._gripperL = baxter_interface.Gripper("left")
        self._gripperR = baxter_interface.Gripper("right")
        # Initialize inverse kinematics service
        self._ikL = rospy.ServiceProxy('/Baxter_IK_left/', BaxterIK)
        self._ikR = rospy.ServiceProxy('/Baxter_IK_right/', BaxterIK)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        # An orientation for gripper fingers to be overhead and parallel to an object - use these values for orientation when you query the IK service 
        self._overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
        self._home_positionL ={'left_w0': -0.1814934773792869, 
                             'left_w1': 0.026825777272033946, 
                             'left_w2': -0.012962544641613505, 
                             'left_e0': -0.016749916819684962, 
                             'left_e1': 0.494483621052642, 
                             'left_s0': 0.19265380402451004, 
                             'left_s1': 1.047000093806477}
        self._home_positionR = {'right_s0': -0.27280529300830825, 
                             'right_s1': 1.0470000138121165, 
                             'right_w0': -0.08776609085386955, 
                             'right_w1': 0.02797871412196784, 
                             'right_w2': 0.02651549204258874, 
                             'right_e0': -0.021192032268176675, 
                             'right_e1': 0.4955784414468667}
        # Start position - use these angles for the robot's start position
        self._start_positionL = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
        self._start_positionR = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': 1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                             'right_s1': -0.9999781166910306}
        self._r_positionL = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': 2,
                             'left_s1': -0.9999781166910306}
        self._r_positionR = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': 1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -2,
                             'right_s1': -0.9999781166910306}
        self._t1 = [0.578, 0.6298, -0.127527951002 ]
        self._t2 = [0.578, -0.569, -0.127527951002 ]
        self._pos = [[0.57, 0.03, -0.127492113709], [0.65, 0.03, -0.127492113709], [0.8, 0.03, -0.127492113709]]   
    def plan_callback(self, data):
       self._plan = data
    def move_to_approach_positionL(self, point):
        point = copy.deepcopy(point)
        pose = Pose(position=point)
        pose.position.z = pose.position.z+0.15
        pose.orientation = self._overhead_orientation
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        srv = BaxterIKRequest()
        srv.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        resp = self._ikL(srv)
        out = dict(zip(resp.joint_angles[0].name, resp.joint_angles[0].position))
        self._limbL.move_to_joint_positions(out)
    def move_to_approach_positionR(self, point):
        point = copy.deepcopy(point)
        pose = Pose(position=point)
        pose.position.z = pose.position.z+0.15
        pose.orientation = self._overhead_orientation
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        srv = BaxterIKRequest()
        srv.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        resp = self._ikR(srv)
        out = dict(zip(resp.joint_angles[0].name, resp.joint_angles[0].position))
        self._limbR.move_to_joint_positions(out)
    def move_to_pickup_positionR(self, point):
        pose = Pose(position=point)
        pose.position.y = pose.position.y
        pose.orientation = self._overhead_orientation
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        srv = BaxterIKRequest()
        srv.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        resp = self._ikR(srv)
        out = dict(zip(resp.joint_angles[0].name, resp.joint_angles[0].position))
        self._limbR.move_to_joint_positions(out)
    def move_to_pickup_positionL(self, point):
        pose = Pose(position=point)
        pose.position.y = pose.position.y
        pose.orientation = self._overhead_orientation
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        srv = BaxterIKRequest()
        srv.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        resp = self._ikL(srv)
        out = dict(zip(resp.joint_angles[0].name, resp.joint_angles[0].position))
        self._limbL.move_to_joint_positions(out)
    def gripL(self):
        self._gripperL.close()
        rospy.sleep(1.0)
    def gripR(self):
        self._gripperR.close()
        rospy.sleep(1.0)
    def ungripL(self):
        self._gripperL.open()
        rospy.sleep(1.0)
    def ungripR(self):
        self._gripperR.open()
        rospy.sleep(1.0)
    def get_blocks(self): 
	gzblock = rospy.ServiceProxy('/gazebo/get_model_state/', GetModelState)
        green = gzblock('block01','world')
        green = [green.pose.position.x+0.02, green.pose.position.y+0.03, green.pose.position.z-0.91] 
	red = gzblock('block02','world')
        red = [red.pose.position.x+0.02, red.pose.position.y+0.03, red.pose.position.z-0.91] 
        blue = gzblock('block03','world')
        blue = [blue.pose.position.x+0.02, blue.pose.position.y+0.03, blue.pose.position.z-0.91] 
        return {'red':red, 'green':green, 'blue':blue}
    def main(self):
        while(True):
            block_locations = self.get_blocks()
            if self._plan:
                if len(self._plan.plan)==0:
                    print "Finished"
                    finished_str = self._ID + ":" + self._name
                    pub_finished = rospy.Publisher('/pickup_finished', String, queue_size=100)
                    rospy.sleep(1) 
                    pub_finished.publish(finished_str)
                    rospy.sleep(0.5)
                    return
                self._limbL.move_to_joint_positions(self._start_positionL)
                self._limbR.move_to_joint_positions(self._start_positionR)
                plan = self._plan.plan[0].step.data.split(' ')
                print(plan)
                if plan[0] == "pickup":
                    block = plan[1]
                    hand = plan[2]
                    for b in block_locations.iteritems():
                        if b[0]==block:
                            p = Point()
                            p.x = b[1][0]
                            p.y = b[1][1]
                            p.z = b[1][2]
                            if hand=="left":
                                self.move_to_approach_positionL(p)
                                self._gripperL.open()
                                self.move_to_pickup_positionL(p)
                                self._gripperL.close()
                                rospy.sleep(0.5)
                                self._limbL.move_to_joint_positions(self._start_positionL)
                            if hand=="right":
                                self.move_to_approach_positionR(p)
                                self._gripperR.open()
                                self.move_to_pickup_positionR(p)
                                self._gripperR.close()
                                rospy.sleep(0.5)
                                self._limbR.move_to_joint_positions(self._start_positionR)
                            self._plan.plan.pop(0)
                plan = self._plan.plan[0].step.data.split(' ')
                print(plan)
                if plan[0] == "place":
                    block = plan[1]
                    pos = plan[2]
                    hand = plan[3]
                    for b in block_locations.iteritems():
                        if b[0]==block:
                            if hand=="left" and pos=="center":
                                p = Point()
                                if block=="red":
                                    p.x = self._pos[0][0]
                                    p.y = self._pos[0][1]
                                    p.z = self._pos[0][2]
                                elif block=="blue":
                                    p.x = self._pos[2][0]
                                    p.y = self._pos[2][1]
                                    p.z = self._pos[2][2]
                                else:
                                    p.x = self._pos[1][0]
                                    p.y = self._pos[1][1]
                                    p.z = self._pos[1][2]
                                self.move_to_pickup_positionL(p)
                                self._gripperL.open()
                                rospy.sleep(0.5)
                                p.z = p.z+0.15
                                self.move_to_pickup_positionL(p)
                                self._limbL.move_to_joint_positions(self._start_positionL)
                                self._plan.plan.pop(0)
                            elif hand=="left":
                                p = Point() 
                                if pos=="t1":
                                    p.x = self._t1[0]
                                    p.y = self._t1[1]
                                    p.z = self._t1[2]
                                if pos=="t2":
                                    p.x = self._t2[0]
                                    p.y = self._t2[1]
                                    p.z = self._t2[2]
                                self.move_to_pickup_positionL(p)
                                self._gripperL.open()
                                rospy.sleep(0.5)
                                p.z = p.z+0.15
                                self.move_to_pickup_positionL(p)
                                self._limbL.move_to_joint_positions(self._start_positionL)
                                self._plan.plan.pop(0)
                            if hand=="right" and pos=="center":
                                p = Point()
                                if block=="red":
                                    p.x = self._pos[0][0]
                                    p.y = self._pos[0][1]
                                    p.z = self._pos[0][2]
                                elif block=="blue":
                                    p.x = self._pos[2][0]
                                    p.y = self._pos[2][1]
                                    p.z = self._pos[2][2]
                                else:
                                    p.x = self._pos[1][0]
                                    p.y = self._pos[1][1]
                                    p.z = self._pos[1][2]
                                self.move_to_pickup_positionR(p)
                                self._gripperR.open()
                                rospy.sleep(0.5)
                                p.z = p.z+0.15
                                self.move_to_pickup_positionR(p)
                                self._limbR.move_to_joint_positions(self._start_positionR)
                                self._plan.plan.pop(0)
                            elif hand=="right":
                                p = Point() 
                                if pos=="t1":
                                    p.x = self._t1[0]
                                    p.y = self._t1[1]
                                    p.z = self._t1[2]
                                if pos=="t2":
                                    p.x = self._t2[0]
                                    p.y = self._t2[1]
                                    p.z = self._t2[2]
                                self.move_to_pickup_positionR(p)
                                self._gripperR.open()
                                rospy.sleep(0.5)
                                p.z = p.z+0.15
                                self.move_to_pickup_positionR(p)
                                self._limbR.move_to_joint_positions(self._start_positionR)
                                self._plan.plan.pop(0)

if __name__ == "__main__":
    student_name = raw_input("Enter the your name (Last, First): ")
    student_ID = raw_input("Enter the UID: ")
    bp = BaxterPickup("left", student_name, student_ID)
    bp.main()
    



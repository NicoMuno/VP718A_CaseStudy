import pybullet as p
import math
import os

from utils.enums import State, Motion

class DemoRobot:
    def __init__(self,x=0,y=0):
        body_length = 1.2
        body_width = 0.8
        body_height = 0.3
        wheel_radius = 0.15
        wheel_width = 0.1

        linearDamping = 0.2
        angularDamping = 0.02
        lateralFriction = 0.1
        rollingFriction = 0.01
        spinningFriction = 0.01

        self.left_speed=0
        self.right_speed=0

        self.max_speed  = 15.0
        self.max_force  = 50.0 

        self.drive_mode = True
        self.mode_change = False
        self.state = State.MANUAL
        self.motion = Motion.STOP
        self.selected = False

        # Create multi-body with wheels 
        self.agv_id = self.build_robot(x,y,body_length,body_width,body_height,wheel_radius,wheel_width)
        # Number of joints
        self.num_joints = p.getNumJoints(self.agv_id)
        self.wheel_joints = list(range(self.num_joints))
        self.right_wheels = [0,2]
        self.left_wheels  = [1,3]
        # Set Dynamics
        self.set_dynamics(linearDamping,angularDamping,lateralFriction,rollingFriction,spinningFriction)

        # Tasks
        self.task_path = f'tasks/robo_{self.agv_id}_task.txt' 
        self.task_motions = []
        self.task_index = 0
        self.recording = False # in main key -> 1
        self.replaying = False # in main key -> 2

        record_dir = os.path.dirname(self.task_path)
        if record_dir:
            os.makedirs(record_dir, exist_ok=True)
        if os.path.exists(self.task_path):
            print(f"[Robot:{self.agv_id}] Found existing recording: {self.task_path}")
        else:
            print(f"[Robot:{self.agv_id}] No recording yet at: {self.task_path}")


    def user_control(self, keys):
        self.update_state_and_color()
        if self.state == State.SLEEP:
            self.left_speed = 0.0
            self.right_speed = 0.0
            return
        
        self.motion = Motion.STOP
        left = 0.0
        right = 0.0
        drive = self.max_speed
        turn = self.max_speed

        if self.state in (State.MANUAL, State.RECORD):
            w = (ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN)
            s = (ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN)
            a = (ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN)
            d = (ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN)

            if w: self.motion = Motion.STRAIGHT_FRONT
            elif s: self.motion = Motion.STRAIGHT_BACK
            elif a: self.motion = Motion.TURN_LEFT
            elif d: self.motion = Motion.TURN_RIGHT
            else: self.motion = Motion.STOP

        # RECORDING TASK OF ROBOT (toggle handled in main)
        if self.recording and self.state != State.RECORD:
            # start recording
            self.state = State.RECORD
            self.task_motions = []
            self.task_index = 0
            print(f"[Robot:{self.agv_id}] RECORDING of task started.")
        
        if (not self.recording) and self.state == State.RECORD:
            # stop recording
            self.state = State.MANUAL
            print(f"[Robot:{self.agv_id}] RECORDING stopped.")
            self.save_task_to_file()
        
        if self.recording:
            self.task_motions.append(self.motion)
        


        # REPLAYING TASK OF ROBOT
        if self.replaying and self.state != State.REPLAY:
            if self.load_task_from_file():
                self.state = State.REPLAY
                self.task_index = 0
                print(f"[Robot:{self.agv_id}] Starting Task.")

        # Get Motions from Task file
        if self.state == State.REPLAY and self.replaying:
            if self.task_index < len(self.task_motions):
                self.motion = self.task_motions[self.task_index]
                self.task_index += 1
            else:
                # finished replay -> stop and go back to manual
                self.left_speed = 0.0
                self.right_speed = 0.0
                print(f"[Robot:{self.agv_id}] Task finished.")
                self.state = State.MANUAL
                self.replaying = False
                self.task_index = 0

        # Drive dynamics
        if self.mode_change:
            if self.drive_mode:
                # linearDamping,angularDamping,lateralFriction,rollingFriction,spinningFriction
                self.set_dynamics(0.15,0.2,2,0.02,0.005)
                self.mode_change = False
            else:
                #Turn dynamcis
                self.set_dynamics(0.1,0.03,0.6,0,0)
                self.mode_change = False

        if self.motion == Motion.STRAIGHT_FRONT or self.motion == Motion.STRAIGHT_BACK:
            if not(self.drive_mode):
                self.mode_change = True
                self.drive_mode = True
            if self.motion == Motion.STRAIGHT_FRONT:
                left += drive
                right += drive
            if self.motion == Motion.STRAIGHT_BACK:
                left -= drive
                right -= drive
        # Turn
        if not (self.motion == Motion.STRAIGHT_FRONT or self.motion == Motion.STRAIGHT_BACK):
            # spin in place
            if(self.drive_mode):
                self.mode_change = True
                self.drive_mode=False
            if self.motion == Motion.TURN_LEFT:
                left += turn
                right -= turn
            elif self.motion == Motion.TURN_RIGHT:
                left -= turn
                right += turn

        self.left_speed = left
        self.right_speed = right



    def step_action(self):
        """Overwrite this to change behaviour"""
        if self.state == State.SLEEP:
            return

        # Left wheels
        for j in self.left_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.left_speed,
                force=self.max_force
            )
        # Right wheels
        for j in self.right_wheels:
            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.right_speed,
                force=self.max_force
            )

    def set_dynamics(self,ld,ad,lf,rf,sf):
        # body damping
        p.changeDynamics(self.agv_id, 
                         -1,
                         linearDamping=ld, 
                         angularDamping=ad)
        


        
        # Set wheel friction
        for joint_id in self.wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.agv_id,
                jointIndex=joint_id,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
            p.changeDynamics(self.agv_id, 
                             joint_id, 
                             lateralFriction=lf,
                             rollingFriction=rf, 
                             spinningFriction=sf
                             )


    def save_task_to_file(self):
        """Save recorded Task to text file."""
        if not self.task_motions:
            print("Nothing to save. Task recording is empty!")
            return
        with open(self.task_path, "w") as f:
            for s in self.task_motions:
                f.write(s.name + "\n")
        print(f"Saved {len(self.task_motions)} steps to {self.task_path}")

    def load_task_from_file(self):
        """Load task from file into task_motions."""
        if not os.path.exists(self.task_path):
            print(f"Task file not found: {self.task_path}")
            return False
        loaded_motions = []
        with open(self.task_path, "r") as f:
            for line in f:
                loaded_motions.append(Motion[line.strip()])
        if not loaded_motions:
            print(f"Task file is empty or invalid: {self.task_path}")
            return False
        self.task_motions = loaded_motions
        print(f"Loaded {len(self.task_motions)} motions from {self.task_path}")
        return True
    
    def update_state_and_color(self):
        if (not self.selected) and (not self.recording) and (not self.replaying):
            self.state = State.SLEEP
        else:
            # if selected and not in special modes -> manual
            if self.state == State.SLEEP:
                self.state = State.MANUAL  # wake up

        # colors
        if self.recording:
            color = [1.0, 0.0, 0.0, 1.0]  # red
        elif self.replaying:
            color = [0.0, 1.0, 0.0, 1.0]  # green
        elif self.selected:
            color = [1.0, 1.0, 0.0, 1.0]  # yellow
        else:
            # sleeping (or not selected)
            color = [0.2, 0.4, 0.8, 1.0]  # blue

        p.changeVisualShape(self.agv_id, -1, rgbaColor=color)


    def build_robot(self,x,y,body_length,body_width,body_height,wheel_radius,wheel_width):
        chassis_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[body_length/2, body_width/2, body_height/2])
        chassis_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[body_length/2, body_width/2, body_height/2],
                                        rgbaColor=[0.2, 0.4, 0.8, 1.0])
        wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_width)
        wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_width,
                                         rgbaColor=[0.3, 0.3, 0.3, 1.0])
        wheel_positions = [
            [ body_length/3,  body_width/2 + wheel_width/2, -(body_height/2)],  # FR
            [ body_length/3, -body_width/2 - wheel_width/2, -(body_height/2)],  # FL
            [-body_length/3,  body_width/2 + wheel_width/2, -(body_height/2)],  # RR
            [-body_length/3, -body_width/2 - wheel_width/2, -(body_height/2)],  # RL
        ]

        return p.createMultiBody(
            baseMass=50,  # 50kg body
            baseCollisionShapeIndex=chassis_collision,
            baseVisualShapeIndex=chassis_visual,
            basePosition=[x, y, wheel_radius + body_height/2],
            linkMasses=[2.0] * 4,
            linkCollisionShapeIndices=[wheel_collision] * 4,
            linkVisualShapeIndices=[wheel_visual] * 4,
            linkPositions=wheel_positions,
            linkOrientations=[p.getQuaternionFromEuler([math.pi/2, 0, math.pi])] *4,
            linkInertialFramePositions=[[0, 0, 0]] * 4,
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
            linkParentIndices=[0] * 4,
            linkJointTypes=[p.JOINT_REVOLUTE] * 4,
            linkJointAxis=[[0, 0, 1]] * 4 
        )
    
    def set_selected(self, selected: bool):
        """
        If Robot is selected -> wake up and update color
        """
        self.selected = selected
        if selected and self.state == State.SLEEP:
            self.state = State.MANUAL
        self.update_state_and_color()
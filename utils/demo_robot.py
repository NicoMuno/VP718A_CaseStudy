import pybullet as p
import math
import os

from utils.enums import State, Motion, ActionMode

class DemoRobot:
    def __init__(self,x=0,y=0, robo_id=99):
        self.robo_id = robo_id
        # Create multi-body with wheels & init dynamics
        # x,y,body_length,body_width,body_height,wheel_radius,wheel_width
        self.agv_id = self._build_robot(x,y,1.2,0.8,0.3,0.15,0.1)
        self.num_joints = p.getNumJoints(self.agv_id)
        self.wheel_joints = list(range(self.num_joints))
        self.right_wheels = [0,2]
        self.left_wheels  = [1,3]
        # linearDamping,angularDamping,lateralFriction,rollingFriction,spinningFriction
        self._set_dynamics(0,0,0,0,0)

        # Vehilce Config
        self.left_speed=0
        self.right_speed=0
        self.max_speed  = 20.0
        self.turn_speed = 30.0
        self.max_force  = 50.0 
        self.state = State.SLEEP
        self._last_action_mode = ActionMode.UNK

        # Lidar Config
        self.lidar_range = 6.0
        self.stop_range = 4
        self.num_beams = 60 # 360/60 -> 6 grad steps
        self._lidar_skip = 0
        self.debug_lidar = False # press L to see debug LIDAR lines
        self._d_lines = []
        
        # human handling
        self._human_ids = set()
        self.human_detected = False

        # vehilce currently selected
        self.paused = False
        self.selected = False

        # latest keys set in main
        self._keys = {}

        # Timing Controll
        self.physics_dt = 1.0 / 240.0
        self.control_hz = 50.0
        self.control_dt = 1.0 / self.control_hz
        self._control_accum = 0.0

        self.lidar_hz = 5.0
        self._lidar_period = 1.0 / self.lidar_hz  # 0.2s
        self._lidar_accum = 0.0

        # Tasks
        self.task_path = f'tasks/robo_{self.robo_id}_task.txt' 
        self.task = []
        self.task_index = 0
        self.recording = False # in main key -> 1
        self.replaying = False # in main key -> 2

        # RECORD
        # self._rec_run_time = 0.0
        self._rec_last_motion = Motion.STOP
        self._rec_ticks = 0

        self._record_start_pose = None

    def set_human_ids(self, human_ids):
        self._human_ids = set(human_ids)

    def set_pause(self, pause: bool):
        """
        Pause behavior depends on current state:
        - REPLAY  -> keep replaying (do not pause)
        - RECORD  -> pause recording (freeze motion, keep state=RECORD, don't accumulate)
        - MANUAL  -> go to SLEEP
        """
        self.paused = pause

        if pause:
            # Stop accepting keys when not in robot control mode
            self._keys = {}

            if self.state == State.REPLAY or self.replaying:
                # Replaying does NOT stop
                return

            if self.state == State.RECORD or self.recording:
                # Pause recording: freeze movement, keep state as RECORD, do NOT stop recording
                self.left_speed = 0.0
                self.right_speed = 0.0
                return

            # MANUAL -> sleep
            self.state = State.SLEEP
            self.left_speed = 0.0
            self.right_speed = 0.0

        else:
            # unpausing: if selected, wake to MANUAL (unless replay/record already active)
            if self.selected and self.state == State.SLEEP:
                self.state = State.MANUAL



    def set_user_keys(self, keys):
        """
        keys set by main
        """
        self._keys = keys

    def control_step(self, dt):
        """
        If paused:
        - if replaying: still run control ticks (robot continues)
        - else: freeze and return (record stays paused, manual sleeps)
        """
        if self.paused and not (self.state == State.REPLAY or self.replaying):
            self.left_speed = 0.0
            self.right_speed = 0.0
            self._update_state_and_color()
            return

        self._update_state_and_color()

        if self.state == State.SLEEP:
            self.left_speed = 0.0
            self.right_speed = 0.0
            return

        self._control_accum += dt
        while self._control_accum >= self.control_dt:
            self._control_accum -= self.control_dt
            self._control_tick()


    def step_action(self):
        """Overwrite this to change behaviour"""
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

    def set_selected(self, selected: bool):
        """
        If Robot is selected -> wake up and update color
        """
        self.selected = selected
        if selected and self.state == State.SLEEP:
            self.state = State.MANUAL
        self._update_state_and_color()

    def set_debug_lidar(self, enabled: bool):
        self.debug_lidar = enabled
        if not enabled:
            for lid in self._d_lines:
                try:
                    p.removeUserDebugItem(lid)
                except:
                    pass
            self._d_lines.clear()

#####################################################################################################
## HELPER FUNCTIONS #################################################################################
#####################################################################################################


    def _detect_human(self) -> bool:
        """
        Use Lidar to detect human
        """
        if not self._human_ids:
            return False

        max_range = self.lidar_range
        pos, orn = p.getBasePositionAndOrientation(self.agv_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        # rays at approx human belly hight 
        ray_z = pos[2] + 0.5

        ray_from = []
        ray_to = []

        for i in range(self.num_beams):
            ang = yaw + (2.0 * math.pi) * (i / self.num_beams)
            dx = math.cos(ang)
            dy = math.sin(ang)
            ray_from.append([pos[0], pos[1], ray_z])
            ray_to.append([pos[0] + dx * max_range, pos[1] + dy * max_range, ray_z])

        results = p.rayTestBatch(ray_from, ray_to)

        if self.debug_lidar:
            for lid in self._d_lines:
                try:
                    p.removeUserDebugItem(lid)
                except:
                    pass
            self._d_lines.clear()

        hit_human = False

        for (frm, to, result) in zip(ray_from, ray_to, results):
            hit_object_id = result[0]
            hit_fraction = result[2]
            hit_pos = result[3]
            end = to if hit_object_id < 0 else hit_pos

            #debug lines toggle with L
            if self.debug_lidar:
                # green if hit human
                # yellow if hit something else
                # black if nothing
                if hit_object_id in self._human_ids and hit_fraction * max_range <= self.stop_range:
                    color = [0, 1, 0]
                elif hit_object_id >= 0:
                    color = [1, 1, 0]
                else:
                    color = [0, 0, 0]
                self._d_lines.append(p.addUserDebugLine(frm, end, color, lineWidth=1, lifeTime=0.2))

            if hit_object_id in self._human_ids:
                dist = hit_fraction * max_range
                if dist <= self.stop_range:
                    hit_human = True

        return hit_human



    def _control_tick(self):
        # default
        motion = Motion.STOP

        # human detection
        if self.state == State.REPLAY and self.replaying:
            # throttle lidar
            self._lidar_accum += self.control_dt
            if self._lidar_accum >= self._lidar_period:
                self._lidar_accum -= self._lidar_period
                detected = self._detect_human()
                self.human_detected = detected


            if self.human_detected:
                # freeze wheels and freeze replay progress
                self._apply_motion(Motion.STOP)
                return

        # REPLAY
        #-- Init
        if self.replaying and self.state != State.REPLAY:
            if self._load_task_from_file():
                if self._replay_start_pose is not None:
                    pos, orn = self._replay_start_pose
                    p.resetBasePositionAndOrientation(self.agv_id, pos, orn)
                    p.resetBaseVelocity(self.agv_id, [0,0,0], [0,0,0])

                self._control_accum = 0.0
                self._last_action_mode = ActionMode.UNK

                self.state = State.REPLAY
                self._replay_index = 0
                motion, self._replay_ticks_remaining = self.task[0]
                # motion,self._replay_remaining = self.task[0]
            
                print(f"[Robot:{self.robo_id}] Starting Task.")
            else:
                self.replaying = False

        ##-- currently replaying
        if self.replaying and self.state == State.REPLAY:
            # task finished
            if self._replay_index >=len(self.task):
                self._finish_replay()
                motion = Motion.STOP
            else:
                motion, _ = self.task[self._replay_index]
                # self._replay_remaining -= self.control_dt
                self._replay_ticks_remaining -= 1

                # if self._replay_remaining <= 0.0:
                #     self._replay_index += 1
                #     if self._replay_index < len(self.task):
                #         _, self._replay_remaining = self.task[self._replay_index]
                #     else:
                #         self._finish_replay()
                if self._replay_ticks_remaining <= 0:
                    self._replay_index += 1
                    if self._replay_index < len(self.task):
                        _, self._replay_ticks_remaining = self.task[self._replay_index]
                    else:
                        self._finish_replay()
        
        # MANUAL/RECORD
        else:       
            if self.state in (State.MANUAL, State.RECORD):
                keys = self._keys
                w = (ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN)
                s = (ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN)
                a = (ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN)
                d = (ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN)

                if w: motion = Motion.STRAIGHT_FRONT
                elif s: motion = Motion.STRAIGHT_BACK
                elif a: motion = Motion.TURN_LEFT
                elif d: motion = Motion.TURN_RIGHT
                else: motion = Motion.STOP

        
        # Handle task recoding (toggle in main)
        # --Init recording
        if self.recording and self.state != State.RECORD:
            self.state = State.RECORD
            self.task = []
            self._rec_last_motion = Motion.STOP
            # self._rec_run_time = 0.0
            self._rec_ticks = 0
            # store starting point
            self._record_start_pose = p.getBasePositionAndOrientation(self.agv_id)
            p.resetBaseVelocity(self.agv_id, [0,0,0], [0,0,0])
            print(f"[Robot:{self.robo_id}] RECORDING of task started.")
        
        # -- finished recording (save task)
        if (not self.recording) and self.state == State.RECORD:
            # save task
            self._flush_record_run()
            self.state = State.MANUAL
            print(f"[Robot:{self.robo_id}] RECORDING stopped.")
            self._save_task_to_file()
        
        # -- save recording step
        if self.state == State.RECORD and self.recording and (not self.paused):
            if motion == self._rec_last_motion:
                # self._rec_run_time += self.control_dt
                self._rec_ticks += 1
            else:
                self._flush_record_run()
                self._rec_last_motion = motion
                # self._rec_run_time = self.control_dt
                self._rec_ticks = 1

        
        self._apply_motion(motion)

    def _apply_motion(self, motion: Motion):
        drive = self.max_speed
        turn = self.turn_speed
        new_mode = self._last_action_mode
        left = 0.0
        right = 0.0

        if motion == Motion.STRAIGHT_FRONT or motion == Motion.STRAIGHT_BACK:
            new_mode = ActionMode.DRIVE
            if motion == Motion.STRAIGHT_FRONT:
                left += drive
                right += drive
            if motion == Motion.STRAIGHT_BACK:
                left -= drive
                right -= drive
        # Turn
        if motion == Motion.TURN_LEFT or motion == Motion.TURN_RIGHT:
            # spin in place
            new_mode = ActionMode.TURN
            if motion == Motion.TURN_LEFT:
                left += turn
                right -= turn
            elif motion == Motion.TURN_RIGHT:
                left -= turn
                right += turn

        # Drive dynamics
        if new_mode != self._last_action_mode:
            self._apply_dynamics_for(new_mode)
            self._last_action_mode = new_mode
        self.left_speed = left
        self.right_speed = right

    def _apply_dynamics_for(self,new_mode: ActionMode):
        if new_mode == ActionMode.DRIVE:
            print("STRAIGHT DRIVING DYNAMICS SET\n")
            # linearDamping,angularDamping,lateralFriction,rollingFriction,spinningFriction
            self._set_dynamics(ld=0.35, ad=0.2, lf=2.0, rf=0.02, sf=0.0)
            # self._set_dynamics(0.15,0.2,2,0.1,0.01)
        elif new_mode == ActionMode.TURN:
            print("TURN DYNAMICS SET\n")
            self._set_dynamics(ld=0.1,ad=0.5,lf=0.8,rf=0,sf=0.001) #_set_dynamics(ld=0.1,ad=0.3,lf=0.8,rf=0,sf=0)
        else:
            print(f"ACTION MODE is {new_mode.name} - NOTHING CHANGED!")
            

    def _flush_record_run(self):
        # if self._rec_run_time > 0.0:
        #     self.task.append((self._rec_last_motion, self._rec_run_time))
        #     self._rec_run_time = 0.0
        if self._rec_ticks > 0:
            self.task.append((self._rec_last_motion, self._rec_ticks))
            self._rec_ticks = 0
    
    def _finish_replay(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        print(f"[Robot:{self.robo_id}] Task finished.")
        self.state = State.SLEEP
        self.replaying = False
        self.human_detected = False
        self._replay_index = 0
        # self._replay_remaining = 0.0

    def _set_dynamics(self,ld,ad,lf,rf,sf):
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


    def _save_task_to_file(self):
        """Save recorded Task to text file."""
        if not self.task:
            print("Nothing to save. Task recording is empty!")
            return
        # with open(self.task_path, "w") as f:
        #     for motion, dur in self.task:
        #         f.write(f"{motion.name} {dur:.3f}\n")
        # print(f"Saved {len(self.task)} segments to {self.task_path}")
        with open(self.task_path, "w") as f:
            
            # start pose
            if self._record_start_pose is not None:
                (pos, orn) = self._record_start_pose
                f.write(f"START {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} {orn[0]:.6f} {orn[1]:.6f} {orn[2]:.6f} {orn[3]:.6f}\n")

            for motion, ticks in self.task:
                f.write(f"{motion.name} {int(ticks)}\n")
        total_ticks = sum(t for _, t in self.task)
        total_time = total_ticks * self.control_dt
        print(f"Saved {len(self.task)} segments ({total_ticks} ticks, {total_time:.2f}s) to {self.task_path}")

    def _load_task_from_file(self):
        """Load task from file into task_motions."""
        if not os.path.exists(self.task_path):
            print(f"Task file not found: {self.task_path}")
            return False
        loaded = []
        start_pose = None

        # with open(self.task_path, "r") as f:
        #     for line in f:
        #         parts = line.strip().split()
        #         if len(parts) != 2:
        #             continue
        #         m = Motion[parts[0]]
        #         t = float(parts[1])
        #         if t > 0:
        #             loaded.append((m, t))

        with open(self.task_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue

                if parts[0] == "START" and len(parts) == 8:
                    px, py, pz = map(float, parts[1:4])
                    ox, oy, oz, ow = map(float, parts[4:8])
                    start_pose = ([px, py, pz], [ox, oy, oz, ow])
                    continue

                if len(parts) != 2:
                    continue

                m = Motion[parts[0]]
                ticks = int(parts[1])
                if ticks > 0:
                    loaded.append((m, ticks))

        if not loaded:
            print(f"Task file is empty or invalid: {self.task_path}")
            return False
        
        self.task = loaded
        self._replay_start_pose = start_pose
        total_ticks = sum(t for _, t in self.task)
        print(f"Loaded {len(self.task)} segments ({total_ticks} ticks, {total_ticks*self.control_dt:.2f}s) from {self.task_path}")
        return True
        # self.task = loaded
        # total = sum(t for _, t in self.task)
        # print(f"Loaded {len(self.task)} segments ({total:.2f}s) from {self.task_path}")
        # return True
    
    def _update_state_and_color(self):
        """
        If paused:
        - REPLAY stays REPLAY (continue)
        - RECORD stays RECORD (but frozen by control_step)
        - MANUAL/SLEEP stays whatever set by set_pause()
        """
        if not self.paused:
            # normal wake/sleep logic
            if (not self.selected) and (not self.recording) and (not self.replaying):
                self.state = State.SLEEP
            else:
                if self.state == State.SLEEP:
                    self.state = State.MANUAL

        # colors
        if self.human_detected:
            color = [0.6, 0.0, 0.8, 1.0]  # purple
        elif self.recording:
            color = [1.0, 0.0, 0.0, 1.0]  # red
        elif self.replaying:
            color = [0.0, 1.0, 0.0, 1.0]  # green
        elif self.selected:
            color = [1.0, 1.0, 0.0, 1.0]  # yellow
        else:
            color = [0.2, 0.4, 0.8, 1.0]  # blue

        p.changeVisualShape(self.agv_id, -1, rgbaColor=color)



    def _build_robot(self,x,y,body_length,body_width,body_height,wheel_radius,wheel_width):
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
    

import pybullet as p
import utils.world as W
import utils.demo_robot as R
import utils.demo_person as P

from utils.enums import ControlMode


class Simulation:
    def __init__(self):
        self.running = False

        # indices
        self.robo_active_idx = 0
        self.human_active_idx = 0

        self.control_mode = ControlMode.CAMERA

    # ------------------------
    # Initialization
    # ------------------------
    def init(self):
        self.wld = W.BaseWorld()
        self.wld.map_from_file("maps/warehouse_0")
        self.wld.texture_walls()

        self.robots = [
            R.DemoRobot(x=3, y=5, robo_id=0),
            R.DemoRobot(x=3, y=11, robo_id=1),
            R.DemoRobot(x=3, y=17, robo_id=2),
            R.DemoRobot(x=3, y=23, robo_id=3),
            R.DemoRobot(x=11, y=14, robo_id=4),
            R.DemoRobot(x=11, y=20, robo_id=5),
        ]

        self.people = [
            P.DemoPerson(x=2, y=2),
        ]

        # register collisions
        for person in self.people:
            for wall_id in self.wld.cuboids:
                person.add_collidable(wall_id)
            for robot in self.robots:
                person.add_collidable(robot.agv_id)

        # register human_ids with robot 
        person_ids = [person.objectId for person in self.people]
        for robot in self.robots:
            robot.set_human_ids(person_ids)
        
        

        # callbacks
        self.wld.add_step_callback(self.step_all)
        self.wld.add_keyboard_callback(self.keyboard_handler)

        # start in camera mode
        self.enter_camera_mode()

        self.running = True
        print("[Simulation] Initialized")

    # ------------------------
    # Main tick
    # ------------------------
    def tick(self):
        if not self.running:
            return
        self.wld.simStep()

    # ------------------------
    # End Simulation
    # ------------------------
    def end(self):
        print("[Simulation] Shutting down...")
        self.running = False
        self.wld.end()

    # ------------------------
    # Simulation step
    # ------------------------
    def step_all(self):
        dt = 1.0 / 240.0

        for r in self.robots:
            r.control_step(dt)
            r.step_action()

        for h in self.people:
            h.step_action()

    # ------------------------
    # Mode management
    # ------------------------
    def deselect_all(self):
        for r in self.robots:
            r.set_selected(False)
            r.set_user_keys({})
        for h in self.people:
            h.set_selected(False)
            h.set_user_keys({})

    def pause_all_robots(self, pause: bool):
        for r in self.robots:
            r.set_pause(pause)

    def enter_camera_mode(self):
        self.control_mode = ControlMode.CAMERA
        self.pause_all_robots(True)
        self.deselect_all()
        self.wld.set_follow_robot(None)
        self.wld.follow_robot = False

    def enter_robot_mode(self):
        self.control_mode = ControlMode.ROBOTS
        self.pause_all_robots(False)
        self.deselect_all()

        self.robots[self.robo_active_idx].set_selected(True)
        self.wld.set_follow_robot(self.robots[self.robo_active_idx].agv_id)

    def enter_human_mode(self):
        self.control_mode = ControlMode.HUMAN
        self.pause_all_robots(True)
        self.deselect_all()

        self.people[self.human_active_idx].set_selected(True)
        self.wld.set_follow_robot(self.people[self.human_active_idx].objectId)

    # ------------------------
    # Keyboard handling
    # ------------------------
    def keyboard_handler(self, keys):
        # quit with SPACE
        if p.B3G_SPACE in keys and (keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED):
            self.running = False
            return

        # camera mode
        if ord('c') in keys and (keys[ord('c')] & p.KEY_WAS_TRIGGERED):
            self.enter_camera_mode()
            return

        # robot mode
        if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
            if self.control_mode != ControlMode.ROBOTS:
                self.enter_robot_mode()
            else:
                self.robots[self.robo_active_idx].set_selected(False)
                self.robo_active_idx = (self.robo_active_idx + 1) % len(self.robots)
                self.robots[self.robo_active_idx].set_selected(True)
                self.wld.set_follow_robot(self.robots[self.robo_active_idx].agv_id)
            return
        
        if ord('l') in keys and (keys[ord('l')] & p.KEY_WAS_TRIGGERED):
            if self.control_mode == ControlMode.ROBOTS:
                rob = self.robots[self.robo_active_idx]
                rob.set_debug_lidar(not rob.debug_lidar)
                print(f"[Simulation] Lidar debug for Robot {rob.robo_id}: {rob.debug_lidar}")
            return

        # human mode
        if ord('h') in keys and (keys[ord('h')] & p.KEY_WAS_TRIGGERED):
            if self.control_mode != ControlMode.HUMAN:
                self.enter_human_mode()
            else:
                self.people[self.human_active_idx].set_selected(False)
                self.human_active_idx = (self.human_active_idx + 1) % len(self.people)
                self.people[self.human_active_idx].set_selected(True)
                self.wld.set_follow_robot(self.people[self.human_active_idx].objectId)
            return

        # Mode-specific controls
        if self.control_mode == ControlMode.ROBOTS:
            rob = self.robots[self.robo_active_idx]

            # forward keys to selected robot for WASD
            rob.set_user_keys(keys)

            # toggle RECORD (1) - mutual exclusion with replay
            if ord('1') in keys and (keys[ord('1')] & p.KEY_WAS_TRIGGERED):
                if not rob.recording:
                    # starting recording stops replay
                    if rob.replaying:
                        rob.replaying = False
                    rob.recording = True
                else:
                    rob.recording = False

            # toggle REPLAY (2) - mutual exclusion with record
            if ord('2') in keys and (keys[ord('2')] & p.KEY_WAS_TRIGGERED):
                if not rob.replaying:
                    # starting replay stops recording
                    if rob.recording:
                        rob.recording = False
                    rob.replaying = True
                else:
                    rob.replaying = False

            # make sure humans don't keep keys
            for h in self.people:
                h.set_user_keys({})

        elif self.control_mode == ControlMode.HUMAN:
            self.people[self.human_active_idx].set_user_keys(keys)
            for r in self.robots:
                r.set_user_keys({})

        else:
            for r in self.robots:
                r.set_user_keys({})
            for h in self.people:
                h.set_user_keys({})


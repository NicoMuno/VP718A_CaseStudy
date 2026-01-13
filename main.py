import pybullet as p
import utils.world as W
import utils.demo_robot as R
import utils.demo_person as P

from utils.enums import ControlMode

wld = W.BaseWorld()
wld.map_from_file("maps/warehouse_0")
wld.texture_walls()

robots = [
    R.DemoRobot(x=4, y=0),
    R.DemoRobot(x=6, y=0),
    R.DemoRobot(x=8, y=0),
]

people = [
    P.DemoPerson(x=0, y=0),
]


for person in people:
    for wall_id in wld.cuboids:
            person.add_collidable(wall_id)
    for r in robots:
        person.add_collidable(r.agv_id)


control_mode = ControlMode.CAMERA
robo_active_idx = 0
human_active_idx = 0


def deselect_all():
    for r in robots:
        r.set_selected(False)
        r.set_user_keys({})  # ensure no WASD affects them

    for h in people:
        h.set_selected(False)
        h.set_user_keys({})  # ensure no WASD affects them


def pause_all_robots(pause: bool):
    for r in robots:
        r.set_pause(pause)


def enter_camera_mode():
    global control_mode
    control_mode = ControlMode.CAMERA

    # Pause robots with your policy (REPLAY continues, RECORD pauses, MANUAL->SLEEP)
    pause_all_robots(True)

    deselect_all()

    # stop following anything
    wld.set_follow_robot(None)
    wld.follow_robot = False  # force follow OFF


def enter_robot_mode():
    global control_mode, robo_active_idx
    control_mode = ControlMode.ROBOTS

    # Unpause robots (resume RECORD/REPLAY; MANUAL wakes when selected)
    pause_all_robots(False)

    deselect_all()
    robots[robo_active_idx].set_selected(True)

    # set follow target to selected robot; actual following depends on 'F' toggle
    wld.set_follow_robot(robots[robo_active_idx].agv_id)


def enter_human_mode():
    global control_mode, human_active_idx
    control_mode = ControlMode.HUMAN

    # Pause robots with your policy
    pause_all_robots(True)

    deselect_all()
    people[human_active_idx].set_selected(True)

    # follow selected human
    wld.set_follow_robot(people[human_active_idx].objectId)


# Start exactly as requested
enter_camera_mode()


def step_all():
    dt = 1.0 / 240.0

    for r in robots:
        r.control_step(dt)
        r.step_action()

    for h in people:
        h.step_action()


wld.add_step_callback(step_all)


def keyboard_handler(keys):
    global robo_active_idx, human_active_idx, control_mode

    # --- C: return to camera mode ---
    if ord('c') in keys and (keys[ord('c')] & p.KEY_WAS_TRIGGERED):
        enter_camera_mode()
        return

    # --- R: robot mode + cycle robots ---
    if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
        if control_mode != ControlMode.ROBOTS:
            enter_robot_mode()
        else:
            robots[robo_active_idx].set_selected(False)
            robots[robo_active_idx].left_speed = 0.0
            robots[robo_active_idx].right_speed = 0.0

            robo_active_idx = (robo_active_idx + 1) % len(robots)
            robots[robo_active_idx].set_selected(True)

            wld.set_follow_robot(robots[robo_active_idx].agv_id)
        return

    # --- H: human mode + cycle humans ---
    if ord('h') in keys and (keys[ord('h')] & p.KEY_WAS_TRIGGERED):
        if control_mode != ControlMode.HUMAN:
            enter_human_mode()
        else:
            people[human_active_idx].set_selected(False)

            human_active_idx = (human_active_idx + 1) % len(people)
            people[human_active_idx].set_selected(True)

            wld.set_follow_robot(people[human_active_idx].objectId)
        return

    # --- Mode-specific controls ---
    if control_mode == ControlMode.ROBOTS:
        robots[robo_active_idx].set_user_keys(keys)

        # ensure humans never move in robot mode
        for h in people:
            h.set_user_keys({})

        # recording/replay only in ROBOTS mode
        if ord('1') in keys and (keys[ord('1')] & p.KEY_WAS_TRIGGERED):
            rob = robots[robo_active_idx]
            if not rob.recording:
                if rob.replaying:
                    rob.replaying = False
                rob.recording = True
            else:
                rob.recording = False

        if ord('2') in keys and (keys[ord('2')] & p.KEY_WAS_TRIGGERED):
            rob = robots[robo_active_idx]
            if not rob.replaying:
                if rob.recording:
                    rob.recording = False
                rob.replaying = True
            else:
                rob.replaying = False

    elif control_mode == ControlMode.HUMAN:
        people[human_active_idx].set_user_keys(keys)

        for r in robots:
            r.set_user_keys({})

    else:
        for r in robots:
            r.set_user_keys({})
        for h in people:
            h.set_user_keys({})


wld.add_keyboard_callback(keyboard_handler)

while True:
    wld.simStep()

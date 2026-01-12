import pybullet as p
import utils.world as W
import utils.demo_robot as R

wld = W.BaseWorld()
wld.map_from_file("maps/warehouse_0")
wld.texture_walls()

robots = [
    R.DemoRobot(x=0, y=0),
    R.DemoRobot(x=4, y=0),
    R.DemoRobot(x=6, y=0),
]

active_idx = 0
wld.follow_robot = True
wld.set_follow_robot(robots[active_idx].agv_id)
robots[0].set_selected(True)

def step_all():
    for rob in robots:
        rob.step_action()

wld.add_step_callback(step_all)

def keyboard_handler(keys):
    global active_idx

    if ord('r') in keys and (keys[ord('r')] & p.KEY_WAS_TRIGGERED):
        robots[active_idx].left_speed = 0
        robots[active_idx].right_speed = 0
        robots[active_idx].set_selected(False)
        if robots[active_idx].recording:
            robots[active_idx].recording = False
        active_idx = (active_idx + 1) % len(robots)
        robots[active_idx].set_selected(True)
        wld.set_follow_robot(robots[active_idx].agv_id)

        print("Selected robot:", active_idx, "agv_id:", robots[active_idx].agv_id)

    # recording
    if ord('1') in keys and (keys[ord('1')] & p.KEY_WAS_TRIGGERED):
        robots[active_idx].recording = not robots[active_idx].recording
        # robots[active_idx].update_state_and_color()
    # replay
    if ord('2') in keys and (keys[ord('2')] & p.KEY_WAS_TRIGGERED):
        robots[active_idx].replaying = not robots[active_idx].replaying
        # robots[active_idx].update_state_and_color()

    robots[active_idx].user_control(keys)

wld.add_keyboard_callback(keyboard_handler)

while True:
    wld.simStep()

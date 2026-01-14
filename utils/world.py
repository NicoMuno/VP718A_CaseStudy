import pybullet as p
import pybullet_data
import time
import math

import os
os.environ['B3_NO_PROFILE'] = '1'
os.environ['BT_DISABLE_PROFILE'] = '1'
os.environ['BT_NO_PROFILE'] = '1'
os.environ['BULLET_NO_PROFILE'] = '1'
os.environ['B3_NO_PROFILE'] = '1'

def get_forward_vector(yaw,pitch):
    yaw_rad = math.radians(yaw+90)
    pitch_rad = math.radians(pitch)
    return  [
            math.cos(pitch_rad) * math.cos(yaw_rad),
            math.cos(pitch_rad) * math.sin(yaw_rad), 
            math.sin(pitch_rad)
        ]
        
def get_right_vector(yaw,pitch):
    """Get right direction from current camera"""
    yaw_rad = math.radians(yaw)
    return [
            math.cos(yaw_rad),
            math.sin(yaw_rad),
            0
        ]
        

class BaseWorld:
    def __init__(self):
        print("PyBullet Version ",p.getAPIVersion())
        
        # Connect to the physics server
        self.client = p.connect(p.GUI,options="--disable_timer --disable_file_caching")  # Use p.DIRECT for headless mode
        
        # p.setPhysicsEngineParameter(enableInternalProfiling=0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        p.setTimeStep(1./240.)
        p.setGravity(0, 0, -9.81)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Enable GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)  # Enable keyboard shortcuts -> 1 w = wireframe mode -> textures disappear, objects

        # Load a plane (ground)
        self.planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(self.planeId, 
                         -1, 
                         lateralFriction=1.0)
        
        
        self.move_speed=0.2 # 0.02

        self.cuboids=[]

        self.step_callbacks=[]

        self.additional_key_callbacks=[]

        # FOLLOW ROBOT WITH CAM
        self.follow_robot = False
        self.follow_robot_id = None
        self.follow_offset = [0.0, 0.0, 0.5]  # look slightly above base (tweak)

        # CAMERA Placement
        p.resetDebugVisualizerCamera(
            cameraDistance=5,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 8]
        )
    
    def set_follow_robot(self, body_id):
        self.follow_robot_id = body_id


    def add_keyboard_callback(self,x):
        self.additional_key_callbacks.append(x)

    def add_step_callback(self,x):
        self.step_callbacks.append(x)

    def add_cuboid(self,dims,position,color,mass=0):
        """Expects 3 tuples, 
                (length, width, height)
                (x,y,z)
                (r,g,b)  # 0,1 range, not 0-255"""
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=dims)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=dims, rgbaColor=color + [1.0])
            
        wall_id = p.createMultiBody(
                baseMass=mass,  # Static
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=position
            )
        if mass==0:
            self.cuboids.append(wall_id)
        return wall_id

    def map_from_file(self,fname,unitwidth=1.0):
        xs=open(fname,"r").read().strip()
        row=0
        for l in xs.split("\n"):
            col=0
            for c in l.strip():
                y=row*unitwidth+unitwidth/2
                x=col*unitwidth+unitwidth/2
                if c=="1":
                    self.add_cuboid((unitwidth/2,unitwidth/2,unitwidth/2),(x,y,unitwidth/2),[0.7,0.7,0.7])
                if c=="2":
                    self.add_cuboid((unitwidth/2,unitwidth/2,unitwidth/2),(x,y,unitwidth/2),[0.7,0.7,0.7],mass=0.5)
                col+=1
            row+=1

    def texture_walls(self):
        texture_id = p.loadTexture("textures/silly.png")
        for cube_id in self.cuboids:
            # Apply the texture
            p.changeVisualShape(cube_id, -1, textureUniqueId=texture_id)

    def camera_movement(self):
        keys = p.getKeyboardEvents()
        cam_info = p.getDebugVisualizerCamera()

        yaw = cam_info[8]
        pitch = cam_info[9]
        dist = cam_info[10]
        cameraTarget = list(cam_info[11])

        moved = False
        zoomed = False
        rotated = False

        # --- Toggle follow on F (one press) ---
        if ord('f') in keys and (keys[ord('f')] & p.KEY_WAS_TRIGGERED):
            self.follow_robot = not self.follow_robot

        # Speeds (tweak)
        move = self.move_speed
        zoom_speed = max(0.05, 0.05 * dist)
        yaw_speed = 0.5
        pitch_speed = 0.5

        # -------- Zoom: PageUp / PageDown --------
        if p.B3G_PAGE_UP in keys and keys[p.B3G_PAGE_UP] & p.KEY_IS_DOWN:
            dist = max(0.01, dist - zoom_speed)
            zoomed = True

        if p.B3G_PAGE_DOWN in keys and keys[p.B3G_PAGE_DOWN] & p.KEY_IS_DOWN:
            dist = min(50.0, dist + zoom_speed)
            zoomed = True

        # -------- Yaw: O / P --------
        if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN:
            yaw -= yaw_speed
            rotated = True

        if ord('p') in keys and keys[ord('p')] & p.KEY_IS_DOWN:
            yaw += yaw_speed
            rotated = True

        # -------- Pitch: U / I --------
        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
            pitch += pitch_speed
            rotated = True

        if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
            pitch -= pitch_speed
            rotated = True

        pitch = max(-89.0, min(-5.0, pitch))

        # -------- If following: target = robot position --------
        if self.follow_robot and self.follow_robot_id is not None:
            pos, _ = p.getBasePositionAndOrientation(self.follow_robot_id)
            cameraTarget = [
                pos[0] + self.follow_offset[0],
                pos[1] + self.follow_offset[1],
                pos[2] + self.follow_offset[2],
            ]
            moved = True  # target changed each frame

        # -------- If NOT following: allow arrow-key panning --------
        if not self.follow_robot:
            forward = get_forward_vector(yaw, 0)
            right = get_right_vector(yaw, 0)

            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                cameraTarget[0] += forward[0] * move
                cameraTarget[1] += forward[1] * move
                moved = True

            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                cameraTarget[0] -= forward[0] * move
                cameraTarget[1] -= forward[1] * move
                moved = True

            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                cameraTarget[0] -= right[0] * move
                cameraTarget[1] -= right[1] * move
                moved = True

            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                cameraTarget[0] += right[0] * move
                cameraTarget[1] += right[1] * move
                moved = True

        # -------- Apply update --------
        if moved or zoomed or rotated:
            p.resetDebugVisualizerCamera(
                cameraDistance=dist,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=cameraTarget
            )

        # Pass keys to other callbacks (robot control etc.)
        for cb in self.additional_key_callbacks:
            cb(keys)


    def simStep(self):
        for i in self.step_callbacks:
            i()
        self.camera_movement()
        p.stepSimulation()
                
        # time.sleep(1./240.)  # 240 Hz

    def end(self):
        p.disconnect(self.client)

import pybullet as p
import os

class DemoPerson:
    def __init__(self, x=0, y=0):
        urdf_path = os.path.abspath("models/bubbleperson.urdf")

        self.z_ground = 0.75
        self.objectId = p.loadURDF(urdf_path, [x, y, self.z_ground])
        self.new_orn = p.getQuaternionFromEuler([0, 0, 0])

        p.changeDynamics(self.objectId, -1, mass=0)
        
        self.enabled = True
        self.selected = False
        self._keys = {}
        self.speed = 0.05  # meters per physics step (simple)

        self.movements = None  # graph-based autopilot (optional)

        self.current_location = (x, y)
        self.active_steps = []
        self.collidable = set()
        self.collision_callback = None

    def step_action(self):
        if not self.enabled:
            return

        # collision check (unchanged)
        for o in self.collidable:
            contact_points = p.getContactPoints(bodyA=o, bodyB=self.objectId)
            if len(contact_points) > 0:
                print("Collision detected!")
                if self.collision_callback is not None:
                    self.collision_callback()

        # only attempt manual movement if selected
        if self.selected:
            self._manual_move()


    def set_enabled(self, enabled: bool):
        self.enabled = enabled
        if not enabled:
            self.selected = False
            self._keys = {}

    def set_user_keys(self, keys):
        self._keys = keys

    def set_selected(self, selected: bool):
        self.selected = selected
        if not selected:
            self._keys = {}
         
    def set_collision_callback(self,f):
        self.collision_callback=f

    def add_collidable(self,new_id):
        self.collidable.add(new_id)
        # p.setCollisionFilterPair(new_id, self.objectId, -1, -1, enableCollision=1)

#####################################################################################################
## HELPER FUNCTIONS #################################################################################
#####################################################################################################

    def _manual_move(self):
        """Very simple WASD movement (not physics-realistic)."""
        if (not self.enabled) or (not self.selected):
            return False

        keys = self._keys
        dx = 0.0
        dy = 0.0

        if ord('w') in keys and (keys[ord('w')] & p.KEY_IS_DOWN):
            dy += self.speed
        if ord('s') in keys and (keys[ord('s')] & p.KEY_IS_DOWN):
            dy -= self.speed
        if ord('a') in keys and (keys[ord('a')] & p.KEY_IS_DOWN):
            dx -= self.speed
        if ord('d') in keys and (keys[ord('d')] & p.KEY_IS_DOWN):
            dx += self.speed

        if dx == 0.0 and dy == 0.0:
            return False

        x, y = self.current_location
        self._set_position(x + dx, y + dy)
        return True

    def _set_position(self, x, y):
        old_pos, old_orn = p.getBasePositionAndOrientation(self.objectId)

        # try new pose
        p.resetBasePositionAndOrientation(self.objectId, [x, y, self.z_ground], self.new_orn)

        # update collision info for the new pose
        p.performCollisionDetection()

        # if we hit any collidable, revert
        for o in self.collidable:
            if p.getContactPoints(bodyA=self.objectId, bodyB=o):
                p.resetBasePositionAndOrientation(self.objectId, old_pos, old_orn)
                p.resetBaseVelocity(self.objectId, [0, 0, 0], [0, 0, 0])
                return False

        # accept move
        p.resetBaseVelocity(self.objectId, [0, 0, 0], [0, 0, 0])
        self.current_location = (x, y)
        return True



import pybullet as pyb

class URControl():
    def __init__(self, bullet_client_id):
        self._bid = bullet_client_id
        self.__init_params()

    def __init_params(self):
        self._ur_id = None
        self._joint_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._joint_indices = [3, 2, 1, 4, 5, 6]
        self._forces = [500, 500, 500, 500, 500, 500]

    def load_model(self, model_file, base_position=[0.0, 0.0, 0.0]):
        self._ur_id = pyb.loadURDF(
            model_file, 
            base_position,
            useFixedBase=True, 
            physicsClientId=self._bid
        )

    def update(self, joint_state):
        self._joint_state = joint_state
        self.__update()

    def __update(self):
        pyb.setJointMotorControlArray(
            self._ur_id,
            self._joint_indices,
            pyb.POSITION_CONTROL,
            targetPositions=self._joint_state,
            forces=self._forces,
            physicsClientId=self._bid
        )
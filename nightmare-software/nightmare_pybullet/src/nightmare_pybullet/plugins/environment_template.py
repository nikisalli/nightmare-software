#!/usr/bin/env python3

from nightmare_pybullet.plugins.environment import Environment as DefaultEnv


class Environment(DefaultEnv):
    def __init__(self, pybullet, **kargs):
        super().__init__(pybullet)

    # override parent method
    def load_environment_via_code(self):
        """
        This method provides the possibility for the user to define an environment via python code
        example:
        self.pb.loadURDF(...)
        self.pb.loadSDF(...)
        self.pb.loadSoftBody(...)
        self.pb.setTimeStep(0.001)
        self.pb.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25) # ? related to soft bodies
        etc...
        """
        rospy.logwarn('loading custom environment via code!')

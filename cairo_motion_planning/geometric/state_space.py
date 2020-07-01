class SE3():
    """
    Could be useful for task space representation i.e. R3 X T^3 which can be mapped to R^3 X Quaternion space.
    """
    pass


class SawyerConfigurationSpace():
    """
    Very specific configuration space according to Sawyer's articulated design. Difficult to apply a generic topology space to complex articulated arm with joint limts.

    Attributes:
        joint_bounds (list): List of joint range limits. 
    """

    def __init__(self):
        self.joint_bounds = [['right_j0', (-3.0503, 3.0503)],
                             ['right_j1', (-3.8095, 2.2736)],
                             ['right_j2', (-3.0426, 3.0426)],
                             ['right_j3', (-3.0439, 3.0439)],
                             ['right_j4', (-2.9761, 2.9761)],
                             ['right_j5', (-2.9761, 2.9761)],
                             ['right_j6', (-3.14, 3.14)],
                             ['right_gripper_l_finger_joint', (0.0, 0.020833)],
                             ['right_gripper_r_finger_joint',
                                 (-0.020833, 0.0)],
                             ['head_pan', (-5.0952, 0.9064)]]

    def get_bounds(self, joint_names=['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']):
        """Summary

        Args:
            joint_names (None, optional): Description

        Returns:
            TYPE: Description
        """
        return [joint_bounds[1] for joint_bounds in self.joint_bounds if joint_bounds[0] in joint_names]

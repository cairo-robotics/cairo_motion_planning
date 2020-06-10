from samplers import UniformSampler
from state_space import SawyerConfigurationSpace
from state_validity import StateValidyChecker
from urdf_parser_py.urdf import URDF

if __name__ == "__main__":
    with open("sawyer_static.urdf", "r",  encoding="utf-8") as f:
        urdf = URDF.from_xml_string(f.read().encode())
        print(urdf.joints[6])
        joint_bounds = [[joint.name, (joint.limit.lower, joint.limit.upper)] for joint in urdf.joints if joint.limit is not None]
    print(joint_bounds)
    scs = SawyerConfigurationSpace()
    sampler = UniformSampler(scs.get_bounds())


    svc = StateValidyChecker()
    for i in range(0, 100000):
        print(sampler.sample())
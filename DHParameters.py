import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R


class DHParameters:
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        # self.create_DH_parameters()

    def create_DH_parameters(self, joints: list, degree):
        """
        Creates the DH parameters for the robot
        units: meters and radians
        """
        if degree:
            joints = list(map(np.deg2rad, joints))
        else:
            print([np.round(v) for v in list(map(np.rad2deg, joints))])
            pass
        if (
            len(joints) < 7
        ):  # if the number of joints is less than 7, fill the rest with zeros
            joints = joints + [0] * (7 - len(joints))
        print(f"Applied joints: {joints}")
        # joints = joints
        # modified DH parameters [a, alpha, d, theta]
        DH = {
            "Franka Panda": {
                "type": "modified",
                "resource": "https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-paramete",
                "DH": np.matrix(
                    [
                        [0, 0, 333 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, joints[1]],
                        [0, np.pi / 2, 316 / 1000.0, joints[2]],
                        [82.5 / 1000.0, np.pi / 2, 0, joints[3]],
                        [-82.5 / 1000.0, -np.pi / 2, 384 / 1000.0, joints[4]],
                        [0, np.pi / 2, 0, joints[5]],
                        [88 / 1000.0, np.pi / 2, 107 / 1000.0, joints[6]],
                    ]
                ),
            },
            "Franka Franka 3": {  # this is identical to Franka Panda
                "type": "modified",
                "resource": "https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters",
                "DH": np.matrix(
                    [
                        [0, 0, 333 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, joints[1]],
                        [0, np.pi / 2, 316 / 1000.0, joints[2]],
                        [82.5 / 1000.0, np.pi / 2, 0, joints[3]],
                        [-82.5 / 1000.0, -np.pi / 2, 384 / 1000.0, joints[4]],
                        [0, np.pi / 2, 0, joints[5]],
                        [88 / 1000.0, np.pi / 2, 107 / 1000.0, joints[6]],
                    ]
                ),
            },
            "Universe Robot 5e": {
                "type": "modified",
                "resource": "https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/",
                "DH": np.matrix(
                    [
                        [0, 0, 0.1625, joints[0]],
                        [0, np.pi / 2, 0, joints[1]],
                        [-0.425, 0, 0, joints[2]],
                        [-0.3922, 0, 0.1333, joints[3]],
                        [0, np.pi / 2, 0.0997, joints[4]],
                        [0, -np.pi / 2, 0.0996, joints[5]],
                    ]
                ),
            },
            "Universe Robot 3e": {
                "type": "modified",
                "resource": "https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/",
                "DH": np.matrix(
                    [
                        [0, 0, 0.15185, joints[0]],
                        [0, np.pi / 2, 0, joints[1]],
                        [-0.24355, 0, 0, joints[2]],
                        [-0.2132, 0, 0.13105, joints[3]],
                        [0, np.pi / 2, 0.08535, joints[4]],
                        [0, -np.pi / 2, 0.0921, joints[5]],
                    ]
                ),
            },
            "Universe Robot 10e": {
                "type": "modified",
                "resource": "https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/",
                "DH": np.matrix(
                    [
                        [0, 0, 0.1807, joints[0]],
                        [0, np.pi / 2, 0, joints[1]],
                        [-0.6127, 0, 0, joints[2]],
                        [-0.57155, 0, 0.17415, joints[3]],
                        [0, np.pi / 2, 0.11985, joints[4]],
                        [0, -np.pi / 2, 0.11655, joints[5]],
                    ]
                ),
            },
            "KUKA iiwa 7": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 340 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, joints[1]],
                        [0, np.pi / 2, 400 / 1000.0, joints[2]],
                        [0, -np.pi / 2, 0, -joints[3]],
                        [0, np.pi / 2, 400 / 1000.0, joints[4]],
                        [0, -np.pi / 2, 0, joints[5]],
                        [0, np.pi / 2, 130 / 1000.0, joints[6]],
                    ]
                ),
            },
            "KUKA iiwa 14": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 360 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, joints[1]],
                        [0, np.pi / 2, 420 / 1000.0, joints[2]],
                        [0, -np.pi / 2, 0, joints[3]],
                        [0, np.pi / 2, 400 / 1000.0, joints[4]],
                        [0, -np.pi / 2, 0, joints[5]],
                        [0, np.pi / 2, 126 / 1000.0, joints[6]],
                    ]
                ),
            },
            "Kinova Gen3": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue",
                "DH": np.matrix(
                    [
                        [0, 0, 284.8 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 1, -np.pi / 2 + joints[1]],
                        [410 / 1000.0, 0, 0, -np.pi / 2 - joints[2]],
                        [0, -np.pi / 2, 314.3 / 1000.0, -joints[3]],
                        [0, np.pi / 2, 0, -joints[4]],
                        [0, -np.pi / 2, 167.4 / 1000.0, -joints[5]],
                    ]
                ),
            },
            "UFACTORY xArm5": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 267 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, np.deg2rad(-79.35) + joints[1]],
                        [289.489 / 1000.0, 0, 0, np.deg2rad(79.35) + joints[2]],
                        [77.5 / 1000.0, -np.pi / 2, 342.5 / 1000.0, joints[3]],
                        [0, np.pi / 2, 0, joints[4]],  # always theta5 = np.pi/2
                        [76 / 1000.0, -np.pi / 2, 97 / 1000.0, joints[5]],
                    ]
                ),
            },
            "UFACTORY xArm6": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 267 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, -np.deg2rad(79.35) + joints[1]],
                        [289.489 / 1000.0, 0, 0, np.deg2rad(79.35) + joints[2]],
                        [77.5 / 1000.0, -np.pi / 2, 342.5 / 1000.0, joints[3]],
                        [0, np.pi / 2, 0, joints[4]],
                        [76 / 1000.0, -np.pi / 2, 97 / 1000.0, joints[5]],
                    ]
                ),
            },
            "UFACTORY xArm7": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 267 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, joints[1]],
                        [0, np.pi / 2, 293 / 1000.0, joints[2]],
                        [52.2 / 1000.0, -np.pi / 2, 0, np.pi - joints[3]],
                        [-77.5 / 1000.0, np.pi / 2, 342.5 / 1000.0, joints[4]],
                        [0, -np.pi / 2, 0, joints[5]],
                        [-76 / 1000.0, np.pi / 2, 97 / 1000.0, np.pi + joints[6]],
                    ]
                ),
            },
            "ABB YuMI IRB": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 305 / 1000.0, joints[0]],
                        [-30 / 1000.0, -np.pi / 2, 0, joints[1]],
                        [30 / 1000.0, np.pi / 2, 251.5 / 1000.0, joints[2]],
                        [40.5 / 1000.0, -np.pi / 2, 0, np.pi / 2 + joints[3]],
                        [-40.5 / 1000.0, np.pi / 2, 265 / 1000.0, joints[4]],
                        [27 / 1000.0, -np.pi / 2, 0, joints[5]],
                        [-27 / 1000.0, np.pi / 2, 36 / 1000.0, joints[6]],
                    ]
                ),
            },
            "ABB CRB 15000": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 265 / 1000.0, joints[0]],
                        [0, -np.pi / 2, 0, -np.pi / 2 + joints[1]],
                        [444 / 1000.0, 0, 0, joints[2]],
                        [110 / 1000.0, -np.pi / 2, 470 / 1000.0, joints[3]],
                        [0, np.pi / 2, 0, joints[4]],
                        [80 / 1000.0, -np.pi / 2, 101 / 1000.0, np.pi + joints[5]],
                    ]
                ),
            },
            "FUNC CRX-5iA": {
                "type": "modified",
                "resource": "",
                "DH": np.matrix(
                    [
                        [0, 0, 0, joints[0]],
                        [0, -np.pi / 2, 0, -np.pi / 2 + joints[1]],
                        [410 / 1000.0, 0, 0, -joints[2]],
                        [0, -np.pi / 2, 430 / 1000.0, -joints[3]],
                        [0, np.pi / 2, -130 / 1000.0, -joints[4]],
                        [0, -np.pi / 2, 145 / 1000.0, -joints[5]],
                    ]
                ),
            },
            "Sawyer": {
                "type": "modified",
                "resource": "https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.models.DH.Sawyer",
                "DH": np.matrix(
                    [
                        [0.081, -np.pi / 2, 0.317, joints[0]],
                        [0, -np.pi / 2, 0.1925, joints[1]],
                        [0, -np.pi / 2, 0.4, joints[3]],
                        [0, -np.pi / 2, 0.1685, joints[4]],
                        [0, -np.pi / 2, 0.4, joints[5]],
                        [0, -np.pi / 2, 0.1363, joints[6]],
                        [0, 0, 0.1338, joints[6]],
                    ]
                ),
            },
            "Yaskawa Motoman HC10": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/ea88ef98a3239e9f2191d147c94210d1b84a126d/robotkinematicscatalogue/inversekinematics/__6DOF/__industrialRobots/Yaskawa_HC10.py#L4",
                "DH": np.matrix(
                    [
                        [0, 0, 0, joints[0]],
                        [0, -np.pi / 2, 0, -np.pi / 2 + joints[1]],
                        [700 / 1000.0, 0, 0, np.pi / 2 - joints[2]],
                        [0, -np.pi / 2, 500 / 1000.0, -joints[3]],
                        [0, np.pi / 2, 162 / 1000.0, -joints[4]],
                        [0, -np.pi / 2, 130 / 1000.0, np.pi - joints[5]],
                    ]
                ),
            },
            "Elite Robots EC66": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/ea88ef98a3239e9f2191d147c94210d1b84a126d/robotkinematicscatalogue/inversekinematics/__6DOF/__collaborativeRobots/Elite_Robots_EC66.py#L7",
                "DH": np.matrix(
                    [
                        [0, 0, 96 / 1000.0, np.pi + joints[0]],
                        [0, np.pi / 2, 0, np.pi + joints[1]],
                        [418 / 1000.0, 0, 0, 0 + joints[2]],
                        [398 / 1000.0, 0, 122 / 1000.0, 0 + joints[3]],
                        [0, -np.pi / 2, 98 / 1000.0, np.pi + joints[4]],
                        [0, np.pi / 2, 89 / 1000.0, np.pi + joints[5]],
                    ]
                ),
            },
            "Elite Robots EC63": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/ea88ef98a3239e9f2191d147c94210d1b84a126d/robotkinematicscatalogue/inversekinematics/__6DOF/__collaborativeRobots/Elite_Robots_EC63.py#L3",
                "DH": np.matrix(
                    [
                        [0, 0, 140 / 1000.0, np.pi + joints[0]],
                        [0, np.pi / 2, 0, np.pi + joints[1]],
                        [270 / 1000.0, 0, 0, joints[2]],
                        [256 / 1000.0, 0, 103.5 / 1000.0, joints[3]],
                        [0, -np.pi / 2, 98 / 1000.0, np.pi + joints[4]],
                        [0, np.pi / 2, 89 / 1000.0, np.pi + joints[5]],
                    ]
                ),
            },
            "Elite Robots CS66": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/ea88ef9/robotkinematicscatalogue/inversekinematics/__6DOF/__collaborativeRobots/Elite_Robots_CS66.py",
                "DH": np.matrix(
                    [
                        [0, 0, 162.5 / 1000.0, 0 + joints[0]],
                        [0, np.pi / 2, 0, np.pi + joints[1]],
                        [427 / 1000.0, 0, 0, joints[2]],
                        [390.5 / 1000.0, 0, 147.5 / 1000.0, joints[3]],
                        [0, -np.pi / 2, 96.5 / 1000.0, joints[4]],
                        [0, np.pi / 2, 92 / 1000.0, np.pi + joints[5]],
                    ]
                ),
            },
            "GCR10-1300": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue/blob/ea88ef98a3239e9f2191d147c94210d1b84a126d/robotkinematicscatalogue/inversekinematics/__6DOF/__collaborativeRobots/Siasun_GCR10_1300.py#L4",
                "DH": np.matrix(
                    [
                        [0, 0, 165 / 1000, np.pi + joints[0]],
                        [0, np.pi / 2, 0, np.pi / 2 + joints[1]],
                        [608 / 1000, 0, 0, 0 + joints[2]],
                        [566 / 1000, 0, 164 / 1000, -np.pi / 2 + joints[3]],
                        [0, -np.pi / 2, 126 / 1000, 0 + joints[4]],
                        [0, np.pi / 2, 113 / 1000, np.pi + joints[5]],
                    ]
                ),
            },
        }
        return DH

    def calculate_matrix(self, type, data):
        """
        Returns the transformation matrix for a single link
        """
        if type == "modified":  # revised DH parameters
            a_i_1, alpha_i_1, d_i, theta_i = data
            return np.array(
                [
                    [np.cos(theta_i), -np.sin(theta_i), 0, a_i_1],
                    [
                        np.cos(alpha_i_1) * np.sin(theta_i),
                        np.cos(alpha_i_1) * np.cos(theta_i),
                        -np.sin(alpha_i_1),
                        -d_i * np.sin(alpha_i_1),
                    ],
                    [
                        np.sin(alpha_i_1) * np.sin(theta_i),
                        np.sin(alpha_i_1) * np.cos(theta_i),
                        np.cos(alpha_i_1),
                        d_i * np.cos(alpha_i_1),
                    ],
                    [0, 0, 0, 1],
                ]
            )
        elif type == "classic":  # classic DH parameters
            alpha, a, d, theta = data
            return np.array(
                [
                    [
                        np.cos(theta),
                        -np.cos(alpha) * np.sin(theta),
                        np.sin(alpha) * np.sin(theta),
                        a * np.cos(theta),
                    ],
                    [
                        np.sin(theta),
                        np.cos(alpha) * np.cos(theta),
                        -np.sin(alpha) * np.cos(theta),
                        a * np.sin(theta),
                    ],
                    [0, np.sin(alpha), np.cos(alpha), d],
                    [0, 0, 0, 1],
                ]
            )
        else:
            raise ValueError("Invalid type")

    def get_transformations(self, joint_angles: list, degree=True):
        DH = self.create_DH_parameters(joint_angles, degree)
        assert self.robot_name in DH.keys(), "Robot not found in DH parameters"

        # print(f"{len(DH)} DH parameters found")
        DH_params = DH[self.robot_name]["DH"]
        DH_type = DH[self.robot_name]["type"]

        T_total = np.identity(4)

        for i in range(DH_params.shape[0]):
            data = DH_params[i, 0], DH_params[i, 1], DH_params[i, 2], DH_params[i, 3]
            T_i = self.calculate_matrix(DH_type, data)
            T_total = np.dot(T_total, T_i)
            # print(f"{i}# {T_total}")
        # print(T_total)
        return T_total.reshape(4, 4)

    def matrix2RXYZ(self, matrix):
        r = R.from_matrix(matrix[:3, :3])
        return r.as_euler("xyz", degrees=False)

    def matrix2TXYZ(self, matrix):
        Tx, Ty, Tz = matrix[:3, 3] * 1000
        return Tx, Ty, Tz

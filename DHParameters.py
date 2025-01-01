import numpy as np
from numpy.linalg import inv


class DHParameters:
    def __init__(self):
        pass
        # self.create_DH_parameters()

    def create_DH_parameters(self, joints: list):
        """
        Creates the DH parameters for the robot
        units: meters and radians
        """
        joints = list(map(np.deg2rad, joints))
        if len(joints) < 7:
            joints = joints + [0] * (7 - len(joints))
        print(f"joints: {joints}")
        # joints = joints
        DH = {
            "Franka Research 3": {
                "type": "modified",
                "resource": "https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-paramete",
                "DH": np.matrix(
                    # [a, d, alpha, theta]
                    [
                        [0, 0.333, 0, joints[0]],
                        [0, 0, -np.pi / 2, joints[1]],
                        [0, 0.316, np.pi / 2, joints[2]],
                        [0.0825, 0, np.pi / 2, joints[3]],
                        [-0.0825, 0.384, -np.pi / 2, joints[4]],
                        [0, 0, np.pi / 2, joints[5]],
                        [0.088, 0, np.pi / 2, joints[6]],
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
            "Kinova Gen3": {
                "type": "modified",
                "resource": "https://github.com/SaltworkerMLU/RobotKinematicsCatalogue",
                "DH": np.matrix(
                    [
                        [0, 0, 284.8 / 1000.0, -joints[0]],
                        [0, -np.pi / 2, -11.75 / 1000.0, joints[1]],
                        [0, np.pi / 2, 420.75 / 1000.0, -joints[2]],
                        [0, -np.pi / 2, -12.75 / 1000.0, joints[3]],
                        [-0.35 / 1000.0, np.pi / 2, 314.35 / 1000.0, -joints[4]],
                        [0, -np.pi / 2, 0, joints[5]],
                        [0, np.pi / 2, 170.35 / 1000.0, -joints[6]],
                    ]
                ),
            },
        }
        return DH

    def get_transformation_matrix(self, type, data):
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
                    [np.cos(theta), -np.cos(alpha) * np.sin(theta), np.sin(alpha) * np.sin(theta), a * np.cos(theta)],
                    [np.sin(theta), np.cos(alpha) * np.cos(theta), -np.sin(alpha) * np.cos(theta), a * np.sin(theta)],
                    [0, np.sin(alpha), np.cos(alpha), d],
                    [0, 0, 0, 1],
                ]
            )
        else:
            raise ValueError("Invalid type")

    def get_transformations(self, robot_name: str, joint_angles: list):
        DH = self.create_DH_parameters(joint_angles)
        assert robot_name in DH.keys(), "Robot not found in DH parameters"

        DH_params = DH[robot_name]["DH"]
        DH_type = DH[robot_name]["type"]

        T_total = np.identity(4)
        for i in range(DH_params.shape[0]):

            data = DH_params[i, 0], DH_params[i, 1], DH_params[i, 2], DH_params[i, 3]

            T_i = self.get_transformation_matrix(DH_type, data)

            T_total = np.dot(T_total, T_i)

        return T_total

    def matrix2RXYZ(self, matrix):
        matrix = matrix[:3, :3]
        beta = np.arctan2(-matrix[2, 0], np.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0]))
        alpha = np.arctan2(matrix[1, 0] / np.cos(beta), matrix[0, 0] / np.cos(beta))
        r = np.arctan2(matrix[2, 1] / np.cos(beta), matrix[2, 2] / np.cos(beta))
        return r, beta, alpha

    def matrix2TXYZ(self, matrix):
        Tx, Ty, Tz = matrix[:3, 3] * 1000
        return Tx, Ty, Tz

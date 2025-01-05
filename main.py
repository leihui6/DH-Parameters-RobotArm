import DHParameters as dh
import numpy as np

np.set_printoptions(suppress=True, precision=6)

if __name__ == "__main__":
    # robot_name = "Universe Robot 3e"
    robot_name = "Universe Robot 5e"
    # robot_name = "Universe Robot 10e"
    # robot_name = "Franka Franka 3"
    # robot_name = "Kinova Gen3"
    # robot_name = "KUKA iiwa 7"
    # robot_name = "KUKA iiwa 14"
    # robot_name = "UFACTORY xArm5"
    # robot_name = "UFACTORY xArm6"
    # robot_name = "UFACTORY xArm7"
    # robot_name = "ABB YuMI IRB"
    # robot_name = "ABB CRB 15000"
    # robot_name = "FUNC CRX-5iA"
    # robot_name = "Sawyer"
    # robot_name = "Yaskawa Motoman HC10"
    # robot_name = "Elite Robots EC66"
    # robot_name = "Elite Robots CS66"
    # robot_name = "Elite Robots EC63"
    dh = dh.DHParameters(robot_name)
    # res = dh.get_transformations([0, 0, 30, 0, 0, 0, 0])
    res = dh.get_transformations([0, 0, 20, 8, 100, 0])
    # res = dh.get_transformations(
    #     [
    #         8.076545179602742e-07,
    #         4.644371924689443e-09,
    #         2.967058895272128,
    #         2.0943951572957644,
    #         0.14128910496295283,
    #         -1.535889631942559,
    #         2.4989871387840483,
    #     ],
    #     degrees=False,
    # )
    r, beta, alpha = dh.matrix2RXYZ(res)
    print(f"Transformation matrix:\n{np.array2string(res, separator=', ')}")
    Tx, Ty, Tz = dh.matrix2TXYZ(res)
    print(f"Tx, Ty, Tz (mm): {Tx:.3f} {Ty:.3f} {Tz:.3f}")
    print(f"Degree: {np.rad2deg(r):.1f} {np.rad2deg(beta):.1f} {np.rad2deg(alpha):.1f}")

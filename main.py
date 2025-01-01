import DHParameters as dh
import numpy as np

np.set_printoptions(suppress=True, precision=6)

if __name__ == "__main__":
    dh = dh.DHParameters()
    # res = dh.get_transformations("Universe Robot 3e", [0, 0, 10, 0, 0, 0])
    res = dh.get_transformations("Universe Robot 5e", [0, 0, 10, 0, 0, 0])
    # res = dh.get_transformations("Kinova Gen3", [10, 0, 0, 0, 0, 0, 0])
    r, beta, alpha = dh.matrix2RXYZ(res)
    print(f"deg: {np.rad2deg(r):.6f} {np.rad2deg(beta):.6f} {np.rad2deg(alpha):.6f}")
    Tx, Ty, Tz = dh.matrix2TXYZ(res)
    print(f"Tx, Ty, Tz (mm): {Tx:.6f} {Ty:.6f} {Tz:.6f}")
    # print(res)

# DH-Parameters-RobotArm

## Description

This repository aims to calculate the forward kinematics of robot arms using the Denavit-Hartenberg parameters.
The supported robot arms can be found in the `DHParametersRobotArm` class.

## Usage

```python
import DHParameters as dh

dh = dh.DHParameters("Universe Robot 5e")
res = dh.get_transformations([0, 0, 20, 8, 100, 0], degrees=True)
r, beta, alpha = dh.matrix2RXYZ(res)
print(f"Transformation matrix:\n{np.array2string(res, separator=', ')}")
Tx, Ty, Tz = dh.matrix2TXYZ(res)
print(f"Tx, Ty, Tz (mm): {Tx:.3f} {Ty:.3f} {Tz:.3f}")
print(f"Degree: {np.rad2deg(r):.1f} {np.rad2deg(beta):.1f} {np.rad2deg(alpha):.1f}")
```

The result will be:

```shell
Applied joints: [0.0, 0.0, 0.3490658503988659, 0.13962634015954636, 1.7453292519943295, 0.0, 0]
19 DH parameters found
Transformation matrix:
[[-0.153322, -0.469472, -0.869534, -0.833347],
 [-0.984808,  0.      ,  0.173648, -0.116005],
 [-0.081523,  0.882948, -0.462339, -0.105719],
 [ 0.      ,  0.      ,  0.      ,  1.      ]]
Tx, Ty, Tz (mm): -833.347 -116.005 -105.719
Degree: 117.6 4.7 -98.8
```

## Thanks

Thanks to [SaltworkerMLU](https://github.com/SaltworkerMLU) and [RoboDk](https://robodk.com/).
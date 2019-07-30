#!/usr/bin/env python

import transforms3d
import numpy as np

t = np.array([-0.000, 0.015, -0.000])
q = np.array([0.500, -0.497, 0.502, -0.501])

R = transforms3d.quaternions.quat2mat(q)

print("t", t)
print("euler", transforms3d.euler.mat2euler(R))
print("RPY", transforms3d.euler.mat2euler(R))
print("YPR", np.flip(transforms3d.euler.mat2euler(R)))

Z = [1,1,1]

A = transforms3d.affines.compose(t, R, Z)

Ainv = np.linalg.inv(A)

tinv, Rinv, Zinv, Sinv = transforms3d.affines.decompose44(Ainv)

print("")

print("Inverse")
print(tinv)
print("RPY", transforms3d.euler.mat2euler(Rinv))
print("YPR", np.flip(transforms3d.euler.mat2euler(Rinv)))



T_d435_t265 = [
        [0.9999383035223944, -0.00897142630665802, 0.006550012120737061, 0.06514996570147791],
        [0.008887890840318317, 0.999880191772739, 0.012673101314314848, -0.10391119825760928],
        [-0.0066629231699145185, -0.01261410363587153, 0.9998982397445737, -0.007995979710731843],
        [0.0, 0.0, 0.0, 1.0]
        ]

T_t265_d435 = [
        [0.9999369365127865, 0.009121388342308094, -0.006551585467000305, -0.06458857996956519],
        [-0.009199557810483755, 0.9998856531897881, -0.01200202821698935, 0.10440755768773025],
        [0.006441361153837826, 0.012061543016490353, 0.9999065096530506, 0.009705174376777191],
        [0.0, 0.0, 0.0, 1.0]
        ]

tt, Rt, _, _ = transforms3d.affines.decompose44(T_t265_d435)

print("")
print("t265 to d435")

print(tt)
print("RPY", transforms3d.euler.mat2euler(Rt))
print("YPR", np.flip(transforms3d.euler.mat2euler(Rt)))



td, Rd, _, _ = transforms3d.affines.decompose44(T_d435_t265)

print("")
print("d435 to t265")

print(td)
print("RPY", transforms3d.euler.mat2euler(Rd))
print("YPR", np.flip(transforms3d.euler.mat2euler(Rd)))





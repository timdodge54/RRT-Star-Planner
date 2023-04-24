"""TODO: Documentation
"""
import mav_sim.chap5.model_coef as M
import numpy as np

Yv = M.A_lat[0, 0]
Yr = M.A_lat[0, 2]
Ydeltar = M.B_lat[0, 1]
Nv = M.A_lat[2, 0]
Nr = M.A_lat[2, 2]
Ndeltar = M.B_lat[2, 1]

not_called_foo = np.roots([1, -(Yv + Nr), (Yv * Nr - Yr * Nv)])
print(not_called_foo)


tmp = np.linalg.eig(M.A_lat)
print(tmp[0])

# AA = np.array([[M.A_lat[0,0], M.A_lat[0,2]],[M.A_lat[2,0], M.A_lat[2,2]]])
AA = np.array([[Yv, Yr], [Nv, Nr]])
tmp = np.linalg.eig(AA)
print(tmp[0])

wn = np.sqrt(Yv * Nr - Yr * Nv)
print("wn=", wn)

pwo = wn / 10.0
print("p_wo=", pwo)

foo_kr = np.roots(
    [Ndeltar ** 2, 2 * (Nr * Ndeltar + Ydeltar * Nv), (Yv ** 2 + Nr ** 2 + 2 * Yr * Nv)]
)
print("kr=", foo_kr)

kr = -(Nr * Ndeltar + Ydeltar * Nv) / (Ndeltar ** 2) + np.sqrt(
    ((Nr * Ndeltar + Ydeltar * Nv) / (Ndeltar ** 2)) ** 2
    - (Yv ** 2 + Nr ** 2 + 2 * Yr * Nv) / (Ndeltar ** 2)
)
print("kr=", kr)

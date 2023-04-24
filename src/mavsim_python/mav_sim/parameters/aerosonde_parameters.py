"""aerosande_parameters.py presents initial parameters for the problem physics.
"""
from typing import Any

import numpy as np
import numpy.typing as npt
from mav_sim.tools.rotations import Euler2Quaternion

######################################################################################
#   Initial Conditions - States defined in Chapters 2 and 3
######################################################################################
#   Initial conditions for MAV
north0: float = 0.0  # initial north position
east0: float = 0.0  # initial east position
down0: float = -100.0  # initial down position
u0: float = 25.0  # initial velocity along body x-axis
v0: float = 0.0  # initial velocity along body y-axis
w0: float = 0.0  # initial velocity along body z-axis
phi0: float = 0.0  # initial roll angle
theta0: float = 0.0  # initial pitch angle
psi0: float = 0.0  # initial yaw angle
p0: float = 0  # initial roll rate
q0: float = 0  # initial pitch rate
r0: float = 0  # initial yaw rate
Va0: float = np.sqrt(u0 ** 2 + v0 ** 2 + w0 ** 2)  # Initial airspeed

#   Quaternion State
e_quat: npt.NDArray[Any] = Euler2Quaternion(phi0, theta0, psi0)
e0: float = e_quat.item(0)
e1: float = e_quat.item(1)  # Also referred to as e_x
e2: float = e_quat.item(2)  # Also referred to as e_y
e3: float = e_quat.item(3)  # Also referred to as e_z


######################################################################################
#   Physical Parameters
######################################################################################
mass: float = 11.0  # kg
Jx: float = 0.8244  # kg m^2
Jy: float = 1.135
Jz: float = 1.759
Jxz: float = 0.1204
S_wing: float = 0.55  # Planform or wing area
b: float = 2.8956  # wingspan
c: float = 0.18994  # mean chord of wing
rho: float = 1.2682  # air density
e: float = 0.9  # Oswald efficiency factor
AR: float = (b ** 2) / S_wing  # Wingspan ratio
gravity: float = 9.81

######################################################################################
#   Longitudinal Coefficients
######################################################################################
# Equation (4.3) - Coefficients for lift force
C_L_0: float = 0.23
C_L_alpha: float = 5.61
C_L_q: float = 7.95
C_L_delta_e: float = 0.13

# Equation (4.4) - Coefficients for drag force
C_D_0: float = 0.043
C_D_alpha: float = 0.03
C_D_q: float = 0.0
C_D_delta_e: float = 0.0135

# Equation (4.5) - Coefficients for longitutidinal moment
C_m_0: float = 0.0135
C_m_alpha: float = -2.74
C_m_q: float = -38.21
C_m_delta_e: float = -0.99

# Sigmoid transition parameters for transitioning between linear and flat plat models
# equation (4.10)
M: float = 50.0  # Transition rate
alpha0: float = 0.47  # Transition value for alpha

# Drag force model
C_D_p: float = 0.0  # Equation (4.11)

######################################################################################
#   Lateral Coefficients
######################################################################################
# Lateral force, Equation (4.14)
C_Y_0: float = 0.0
C_Y_beta: float = -0.98
C_Y_p: float = 0.0
C_Y_r: float = 0.0
C_Y_delta_a: float = 0.075
C_Y_delta_r: float = 0.19

# Roll moment, Equation (4.15)
C_ell_0: float = 0.0
C_ell_beta: float = -0.13
C_ell_p: float = -0.51
C_ell_r: float = 0.25
C_ell_delta_a: float = 0.17
C_ell_delta_r: float = 0.0024

# Yaw moment, Equation (4.16)
C_n_0: float = 0.0
C_n_beta: float = 0.073
C_n_p: float = 0.069
C_n_r: float = -0.095
C_n_delta_a: float = -0.011
C_n_delta_r: float = -0.069

######################################################################################
#   Propeller thrust / torque parameters (see Section 4.3)
######################################################################################
# Prop parameters
D_prop: float = 20 * (0.0254)  # prop diameter in m

# Motor parameters
KV: float = 145.0  # from datasheet RPM/V - keep in mind that this is the RC
# aircraft motor datasheet version and has different units from
# that in the book (i.e., V-sec/rad)
KQ: float = (1.0 / KV) * 60.0 / (2.0 * np.pi)  # KQ in N-m/A, V-s/rad
R_motor: float = 0.042  # ohms
i0: float = 1.5  # no-load (zero-torque) current (A)


# Inputs
ncells: float = 12.0
V_max: float = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit, Equations (4.17) and (4.18)
C_Q2: float = -0.01664
C_Q1: float = 0.004970
C_Q0: float = 0.005230
C_T2: float = -0.1079
C_T1: float = -0.06044
C_T0: float = 0.09357

######################################################################################
#   Calculation Variables
######################################################################################
#   gamma parameters pulled from (3.13)
gamma: float = Jx * Jz - (Jxz ** 2)
gamma1: float = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2: float = (Jz * (Jz - Jy) + (Jxz ** 2)) / gamma
gamma3: float = Jz / gamma
gamma4: float = Jxz / gamma
gamma5: float = (Jz - Jx) / Jy
gamma6: float = Jxz / Jy
gamma7: float = ((Jx - Jy) * Jx + (Jxz ** 2)) / gamma
gamma8: float = Jx / gamma

#   C values defined on page 69 of Chapter 5.1
C_p_0: float = gamma3 * C_ell_0 + gamma4 * C_n_0
C_p_beta: float = gamma3 * C_ell_beta + gamma4 * C_n_beta
C_p_p: float = gamma3 * C_ell_p + gamma4 * C_n_p
C_p_r: float = gamma3 * C_ell_r + gamma4 * C_n_r
C_p_delta_a: float = gamma3 * C_ell_delta_a + gamma4 * C_n_delta_a
C_p_delta_r: float = gamma3 * C_ell_delta_r + gamma4 * C_n_delta_r
C_r_0: float = gamma4 * C_ell_0 + gamma8 * C_n_0
C_r_beta: float = gamma4 * C_ell_beta + gamma8 * C_n_beta
C_r_p: float = gamma4 * C_ell_p + gamma8 * C_n_p
C_r_r: float = gamma4 * C_ell_r + gamma8 * C_n_r
C_r_delta_a: float = gamma4 * C_ell_delta_a + gamma8 * C_n_delta_a
C_r_delta_r: float = gamma4 * C_ell_delta_r + gamma8 * C_n_delta_r

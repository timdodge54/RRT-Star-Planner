"""control_parameter.py Defines the parameters used in control of the MAV
"""
import mav_sim.chap5.model_coef as TF
import mav_sim.parameters.aerosonde_parameters as MAV

gravity: float = MAV.gravity  # gravity constant
rho: float = MAV.rho  # density of air
sigma: float = 0.05  # low pass filter gain for derivative
Vg: float = TF.Va_trim

# ----------roll loop-------------  Section 6.1.1.1
# get transfer function data for delta_a to phi
wn_roll: float = 10.0  # 20 #7
zeta_roll: float = 0.707
# a_phi_1: float = 1/2 * rho * Vg**2 * MAV.S_wing * MAV.b * MAV.C_p_p * MAV.b / (2 * Vg)
# a_phi_2: float = 1/2 * rho * Vg**2 * MAV.S_wing * MAV.b**2 * MAV.C_p_delta_a
roll_kp: float = wn_roll ** 2 / TF.a_phi2
roll_kd: float = (2 * zeta_roll * wn_roll - TF.a_phi1) / TF.a_phi2

# ----------course loop------------- Section 6.1.1.2
wn_course: float = wn_roll / 20.0
zeta_course: float = 1.0
course_kp: float = (2 * zeta_course * wn_course * Vg) / gravity
course_ki: float = wn_course ** 2 * Vg / gravity

# ----------yaw damper------------- Section 6.1.1.4
yaw_damper_p_wo: float = 0.45
yaw_damper_kr: float = 0.2

# ----------pitch loop------------- Section 6.1.2.1
wn_pitch: float = 15.0
zeta_pitch: float = 0.707
# a_theta_1 = (rho* Vg**2 * MAV.S_wing * MAV.c**2 * MAV.C_m_q) / (2 * MAV.Jy) / (2 * Vg)
# a_theta_2 = (rho * Vg**2*MAV.c* MAV.C_m_alpha) / (2 * MAV.Jy)
# a_theta_3 = (rho * Vg**2 * MAV.S_wing * MAV.c * MAV.C_m_delta_e) / (2 * MAV.Jy)
pitch_kp: float = (wn_pitch ** 2 - TF.a_theta2) / TF.a_theta3
pitch_kd: float = (2 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
wn_theta_squared = wn_pitch ** 2
K_theta_DC: float = (pitch_kp * TF.a_theta3) / (TF.a_theta2 + pitch_kp * TF.a_theta3)

# ----------altitude loop------------- Section 6.1.2.2
wn_altitude: float = wn_pitch / 30.0
zeta_altitude: float = 1.0
altitude_ki: float = wn_altitude ** 2 / K_theta_DC / Vg
altitude_kp: float = (2 * zeta_altitude * wn_altitude) / (K_theta_DC * Vg)
altitude_zone: float = 10.0  # moving saturation limit around current altitude

# ---------airspeed hold using throttle--------------- Section 6.1.2.3
wn_airspeed_throttle: float = 1.5
zeta_airspeed_throttle: float = 2.0
airspeed_throttle_ki: float = wn_airspeed_throttle ** 2 / TF.a_V2
airspeed_throttle_kp: float = (
    2 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1
) / TF.a_V2

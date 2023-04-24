from mav_sim.chap5.compute_models import compute_ss_model
from mav_sim.chap3.mav_dynamics import DynamicState
import numpy as np
from mav_sim.chap5.trim import compute_trim

if __name__ == "__main__":
    # Compute the trim state and input
    state = DynamicState()
    Va_trim = 25.0
    gamma_trim = 0.0
    trim_state, trim_input = compute_trim(state.convert_to_numpy(), Va_trim, gamma_trim)

    # Compute A_lon and A_lat
    A_lon, B_lon, A_lat, B_lat = compute_ss_model(trim_state, trim_input)

    # Compute eigenvalues
    eigvalue_A_lon, eigvec_A_lon = np.linalg.eig(A_lon)
    print(eigvalue_A_lon)

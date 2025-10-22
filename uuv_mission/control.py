# PD Controller for depth regulation of an underwater vehicle
# Mathematically, u[t] = Kp * e[t] + Kd * (e[t] - e[t-1])
def controller(reference: float, depth: float, reference_prev: float, depth_prev: float, Kp: float, Kd: float) -> float:
    error = reference - depth
    error_prev = reference_prev - depth_prev
    derivative = error - error_prev
    control_signal = Kp * error + Kd * derivative   # Main PD controller formula
    return control_signal
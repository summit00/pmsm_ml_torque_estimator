# load_profiles.py
import numpy as np


def constant_load(T=1.0):
    """Constant load torque."""
    return lambda t: T


def step_load(T_low=0.0, T_high=2.0, t_step=0.2):
    """Step in load torque at t_step."""
    def f(t):
        return T_low if t < t_step else T_high
    return f


def ramp_load(T_start=0.0, T_end=2.0, t_ramp=1.0):
    """Linear ramp in load torque over [0, t_ramp]."""
    def f(t):
        if t <= 0.0:
            return T_start
        if t >= t_ramp:
            return T_end
        return T_start + (T_end - T_start) * (t / t_ramp)
    return f


def sinusoidal_load(T_mean=1.0, T_amp=0.5, freq=1.0):
    """Sinusoidal load torque."""
    def f(t):
        return T_mean + T_amp * np.sin(2 * np.pi * freq * t)
    return f


def random_step_load(step_duration=0.05, T_min=0.0, T_max=2.0, seed=None):
    """
    Piecewise constant random load that changes every 'step_duration'.
    """
    rng = np.random.default_rng(seed)
    state = {
        "current_T": rng.uniform(T_min, T_max),
        "last_change": 0.0,
    }

    def f(t):
        if t - state["last_change"] > step_duration:
            state["current_T"] = rng.uniform(T_min, T_max)
            state["last_change"] = t
        return state["current_T"]

    return f

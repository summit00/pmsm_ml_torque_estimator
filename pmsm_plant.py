# pmsm_plant.py
import numpy as np


class PmsmPlant:
    """
    Simple PMSM dq model + mechanical equation.
    State vector: x = [i_d, i_q, omega_m]
    u: [v_d, v_q]
    """

    def __init__(
        self,
        Rs=0.1,       # stator resistance [ohm]
        Ld=1e-3,      # d-axis inductance [H]
        Lq=1.2e-3,    # q-axis inductance [H]
        psi_f=0.05,   # PM flux linkage [Wb]
        p=3,          # pole pairs
        J=0.01,       # inertia [kg*m^2]
        B=0.001,      # viscous friction [N*m*s]
        load_torque_func=None,  # function T_load(t)
    ):
        self.Rs = Rs
        self.Ld = Ld
        self.Lq = Lq
        self.psi_f = psi_f
        self.p = p
        self.J = J
        self.B = B

        if load_torque_func is None:
            self.load_torque_func = lambda t: 0.0
        else:
            self.load_torque_func = load_torque_func

    def ode(self, t, x, u):
        """
        Continuous-time dynamics.
        x: [i_d, i_q, omega_m]
        u: [v_d, v_q]
        Returns dx/dt as np.array.
        """
        i_d, i_q, omega_m= x
        v_d, v_q = u

        # electrical speed
        omega_e = self.p * omega_m

        # electrical dynamics
        di_d_dt = (v_d - self.Rs * i_d + omega_e * self.Lq * i_q) / self.Ld
        di_q_dt = (v_q - self.Rs * i_q - omega_e * self.Ld * i_d - omega_e * self.psi_f) / self.Lq

        # electromagnetic torque
        torque_e = 1.5 * self.p * (self.psi_f * i_q + (self.Ld - self.Lq) * i_d * i_q)

        # mechanical dynamics
        torque_load = self.load_torque_func(t)
        domega_m_dt = (torque_e - torque_load - self.B * omega_m) / self.J
        return np.array([di_d_dt, di_q_dt, domega_m_dt])

    def output(self, t, x):
        """
        Return measured/derived outputs as a dict for logging.
        """
        i_d, i_q, omega_m = x
        omega_e = self.p * omega_m
        torque_e = 1.5 * self.p * (self.psi_f * i_q + (self.Ld - self.Lq) * i_d * i_q)
        torque_load = self.load_torque_func(t)

        return {
            "i_d": i_d,
            "i_q": i_q,
            "omega_m": omega_m,
            "omega_e": omega_e,
            "torque_e": torque_e,
            "torque_load": torque_load,
        }

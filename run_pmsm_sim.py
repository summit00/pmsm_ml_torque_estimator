# run_pmsm_sim.py
from pmsm_plant import PmsmPlant
from foc_controller import FocCurrentController
from speed_controller import SpeedController
from load_profiles import step_load
from pmsm_simulator import PmsmFocSimulator
from pmsm_plotter import plot_pmsm_results
import numpy as np
import time


def iq_ref_profile(t: float) -> float:
    # if t < 0.05:
    #     return 1.0
    # elif t < 0.40:
    #     return 5.0
    # elif t < 0.80:
    #     return 3.0
    # else:
    return 0.0
        # Updated to clarify the transition to omega phase
        # The iq_ref_profile will now stop at 0.40s and omega_ref_profile will take over.


def omega_ref_profile(t: float) -> float:
    if t < 0.20:
        return 10.0
    elif t < 0.40:
        return 50.0
    elif t < 0.80:
        return 100.0
    elif t < 1.00:
        return 50.0
    else:
        return 0.0
        # Updated to clarify that omega_ref_profile is only active after 0.40s


def main():
    t_final = 1.5  # extended to fit all steps
    dt_sim=1e-6
    dt_current=5e-5
    dt_speed=2.5e-4

    # Motor & FOC parameters (shared for plant + controller)
    Rs = 0.315
    Ld = 3e-4
    Lq = 2.8e-4
    psi_f = 0.0107
    p = 3
    J = 0.0000075
    B = 0

    # define load torque profile
    load_func = step_load(T_low=0.0, T_high=0.0, t_step=0.15)

    # Plant
    plant = PmsmPlant(
        Rs=Rs,
        Ld=Ld,
        Lq=Lq,
        psi_f=psi_f,
        p=p,
        J=J,
        B=B,
        load_torque_func=load_func,
    )

    # Speed controller (PI)
    speed_ctrl = SpeedController(
        dt=dt_speed,
        kp_w=0.02,
        ki_w=0.075,
        iq_limit=5.0,
    )

    # FOC current controller
    foc = FocCurrentController(
        dt=dt_current,
        kp_d=0.4,
        ki_d=300,
        kp_q=0.4,
        ki_q=300,
        Ld=Ld,
        Lq=Lq,
        psi_f=psi_f,
        p=p,
        v_limit=24.0,
    )

        # --- Single simulation: iq steps, stop, then omega steps ---
    sim = PmsmFocSimulator(
            plant=plant,
            current_controller=foc,
            t_final=t_final,
            iq_ref_func=iq_ref_profile,
            id_ref_func=lambda t: 0.0,
            omega_ref_func=omega_ref_profile,
            speed_controller=speed_ctrl,
            dt_sim=dt_sim,
            dt_current=dt_current,
            dt_speed=dt_speed,
    )
    start = time.perf_counter()
    df = sim.run()
    end = time.perf_counter()
    print(f"Simulation time: {end - start:.3f} s")
    print("Data Collected: ", len(df), "rows")
    print(df.head())
    plot_pmsm_results(df.iloc[::10])  # plot every 10th sample


if __name__ == "__main__":
    main()

"""Line-for-line Python port of scripts/P4_Hybrid_Architectural_Transition.m.

Pure standard library. Usage: python -I p4_port.py <uddsdc.csv> <out_dir>
Writes p4_timeseries.csv and p4_metrics.txt into out_dir.
"""
import csv
import math
import sys
from pathlib import Path

src, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)

# PART 1: parameters (identical to the .m file)
m, g, Crr, rho, Cd, A, r_wheel = 1800, 9.81, 0.01, 1.225, 0.29, 2.2, 0.3
P_ice_max, G_front, ice_eff, LHV = 100e3, 4.0, 0.30, 44e6
T_motor_max, T_regen_max, P_motor_max, G_rear, v_min_regen = 250, 150, 80e3, 8.5, 1.5
Batt_capacity_kWh, P_accessory = 15, 900

# PART 2: UDDS (row 1 "UDDS,", row 2 header names, then numeric rows)
t, v = [], []
with src.open(encoding="utf-8-sig") as f:
    for row in csv.reader(f):
        try:
            t.append(float(row[0])); v.append(float(row[1]) * 0.44704)
        except (ValueError, IndexError):
            continue
N = len(t)
dt = sum(t[i + 1] - t[i] for i in range(N - 1)) / (N - 1)
a = [(v[i + 1] - v[i]) / dt for i in range(N - 1)] + [0.0]
F_total = [m * g * Crr + 0.5 * rho * Cd * A * v[i] ** 2 + m * a[i] for i in range(N)]

# PART 3: simulation loop
F_ice = [0.0] * N; F_em = [0.0] * N; P_em_batt = [0.0] * N; P_fuel_W = [0.0] * N
SOC = [0.0] * N; SOC[0] = 0.80
mode = [0] * N  # 1 EV, 2 hybrid, -1 regen, 0 friction/none
omega_max_motor = max(vi / r_wheel * G_rear for vi in v) + 2.220446049250313e-16
F_em_cap = (T_motor_max * G_rear) / r_wheel

for i in range(N - 1):
    if F_total[i] > 0:
        if SOC[i] > 0.25 and v[i] < 15:
            F_em[i] = min(F_total[i], F_em_cap)
            F_ice[i] = max(0.0, F_total[i] - F_em[i])
            mode[i] = 1
        else:
            F_ice[i] = F_total[i] * 0.7
            F_em[i] = F_total[i] * 0.3
            mode[i] = 2
    else:
        if v[i] > v_min_regen:
            F_em[i] = max(F_total[i], -(T_regen_max * G_rear) / r_wheel)
            mode[i] = -1
        else:
            F_em[i] = 0.0
        F_ice[i] = 0.0

    w_m = (v[i] / r_wheel) * G_rear
    T_m = (F_em[i] * r_wheel) / G_rear
    Tn = abs(T_m) / T_motor_max
    on = w_m / omega_max_motor
    if F_em[i] >= 0:
        eff = min(max(0.90 - 0.15 * (1 - on) ** 2 - 0.20 * (Tn - 0.6) ** 2, 0.75), 0.92)
        P_em_batt[i] = (F_em[i] * v[i]) / eff
    else:
        eff = min(max(0.75 - 0.20 * (1 - on) ** 2 - 0.30 * (Tn - 0.5) ** 2, 0.55), 0.80)
        P_em_batt[i] = (F_em[i] * v[i]) * eff

    P_fuel_W[i] = F_ice[i] * v[i] / ice_eff

    P_batt = P_em_batt[i] + P_accessory
    SOC[i + 1] = SOC[i] - ((P_batt * dt) / (3600 * 1000)) / Batt_capacity_kWh
    SOC[i + 1] = min(max(SOC[i + 1], 0.0), 1.0)


def trapz(y):
    return sum((t[i + 1] - t[i]) * (y[i] + y[i + 1]) / 2 for i in range(N - 1))


def cumtrapz(y):
    c = [0.0]
    for i in range(N - 1):
        c.append(c[-1] + (t[i + 1] - t[i]) * (y[i] + y[i + 1]) / 2)
    return c


# PART 4: script's own metrics
dist_km = trapz(v) / 1000
Fuel_L = trapz(P_fuel_W) / LHV / 0.74

# Derived metrics (same model, no new assumptions except the ICE-only reference)
P_wheel = [F_total[i] * v[i] for i in range(N)]
P_ice_mech = [F_ice[i] * v[i] for i in range(N)]
P_em_mech = [F_em[i] * v[i] for i in range(N)]
P_fuel_ref = [max(p, 0.0) / ice_eff for p in P_wheel]  # conventional car, same 30 % BTE
Fuel_ref_L = trapz(P_fuel_ref) / LHV / 0.74
cum_h = [x / LHV / 0.74 for x in cumtrapz(P_fuel_W)]
cum_r = [x / LHV / 0.74 for x in cumtrapz(P_fuel_ref)]
kwh = 3.6e6
E_wheel_pos = trapz([max(p, 0.0) for p in P_wheel]) / kwh
E_brake = -trapz([min(p, 0.0) for p in P_wheel]) / kwh
E_ice = trapz(P_ice_mech) / kwh
E_em_drive = trapz([max(p, 0.0) for p in P_em_mech]) / kwh
E_batt_out = trapz([max(p, 0.0) for p in P_em_batt]) / kwh
E_regen = -trapz([min(p, 0.0) for p in P_em_batt]) / kwh
E_acc = P_accessory * (t[-1] - t[0]) / kwh
moving = [i for i in range(N - 1) if v[i] > 0 and F_total[i] > 0]
ev_share = 100 * sum(mode[i] == 1 for i in moving) / len(moving)
hyb_share = 100 * sum(mode[i] == 2 for i in moving) / len(moving)
w_m_all = [(v[i] / r_wheel) * G_rear for i in range(N)]
T_m_all = [(F_em[i] * r_wheel) / G_rear for i in range(N)]
dur = t[-1] - t[0]

lines = [
    f"Duration_s {dur:.0f}",
    f"Distance_km {dist_km:.3f}",
    f"Mean_speed_kmh {dist_km / (dur / 3600):.2f}",
    f"Max_speed_kmh {max(v) * 3.6:.2f}",
    f"SOC_start_pct {SOC[0] * 100:.2f}",
    f"SOC_end_pct {SOC[-1] * 100:.3f}",
    f"Fuel_hybrid_L {Fuel_L:.4f}",
    f"Fuel_hybrid_L100km {Fuel_L / dist_km * 100:.3f}",
    f"Fuel_ref_L {Fuel_ref_L:.4f}",
    f"Fuel_ref_L100km {Fuel_ref_L / dist_km * 100:.3f}",
    f"Fuel_saving_pct {(1 - Fuel_L / Fuel_ref_L) * 100:.2f}",
    f"EV_share_moving_pct {ev_share:.2f}",
    f"Hybrid_share_moving_pct {hyb_share:.2f}",
    f"Regen_time_pct {100 * sum(md == -1 for md in mode) / (N - 1):.2f}",
    f"E_wheel_pos_kWh {E_wheel_pos:.4f}",
    f"E_ice_mech_kWh {E_ice:.4f}",
    f"ICE_share_of_wheel_pct {100 * E_ice / E_wheel_pos:.2f}",
    f"E_em_drive_kWh {E_em_drive:.4f}",
    f"EM_share_of_wheel_pct {100 * E_em_drive / E_wheel_pos:.2f}",
    f"E_batt_out_drive_kWh {E_batt_out:.4f}",
    f"E_regen_to_batt_kWh {E_regen:.4f}",
    f"E_brake_wheel_kWh {E_brake:.4f}",
    f"Regen_recovery_pct {100 * E_regen / E_brake:.2f}",
    f"E_accessory_kWh {E_acc:.4f}",
    f"Peak_P_ice_kW {max(P_ice_mech) / 1e3:.2f}",
    f"Peak_P_em_drive_kW {max(P_em_mech) / 1e3:.2f}",
    f"Peak_P_em_regen_kW {-min(P_em_mech) / 1e3:.2f}",
    f"Peak_T_m_drive_Nm {max(T_m_all):.1f}",
    f"Peak_T_m_regen_Nm {-min(T_m_all):.1f}",
    f"Peak_w_m_rads {max(w_m_all):.1f}",
    f"EV_force_cap_N {F_em_cap:.0f}",
    f"Peak_F_total_N {max(F_total):.0f}",
]
(out / "p4_metrics.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))

with (out / "p4_timeseries.csv").open("w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["t", "v_kmh", "F_ice", "F_em", "P_ice_kW", "P_em_kW", "P_batt_kW",
                "SOC", "fuel_h", "fuel_ref", "mode", "w_m", "T_m"])
    for i in range(N):
        w.writerow([f"{t[i]:.0f}", f"{v[i] * 3.6:.3f}", f"{F_ice[i]:.2f}", f"{F_em[i]:.2f}",
                    f"{P_ice_mech[i] / 1e3:.4f}", f"{P_em_mech[i] / 1e3:.4f}",
                    f"{(P_em_batt[i] + P_accessory) / 1e3:.4f}", f"{SOC[i] * 100:.4f}",
                    f"{cum_h[i]:.5f}", f"{cum_r[i]:.5f}", mode[i],
                    f"{w_m_all[i]:.3f}", f"{T_m_all[i]:.3f}"])

#!/usr/bin/env python3
"""Torque margin in a standing tripod, servo current, regulator and battery sizing. Uses the YAML and the gait generator."""
import numpy as np

from hexapod_kinematics import GaitParams, RobotModel, VelocityCommand, feet_fk, generate_gait, leg_fk, leg_ik

m = RobotModel.load()
g = 9.80665
stall = m.servo['stall_torque_6v']
i_stall = m.servo['stall_current_6v']
I_IDLE = 0.17   # A, no-load current quoted by one source; EST
W = m.total_mass * g
print(f'Robot {m.total_mass * 1000:.0f} g, weight {W:.1f} N, MG996R stall {stall:.2f} N.m at 6 V')


def torques(q, feet=3):
    """Static joint torques from the foot load. Each foot carries W / feet; link masses are small next to that."""
    load = W / feet
    femur_arm = m.femur * np.cos(q[1]) + m.tibia * np.sin(q[1] + q[2])
    tibia_arm = m.tibia * np.sin(q[1] + q[2])
    return load * abs(femur_arm), load * abs(tibia_arm), femur_arm, tibia_arm


def current(tau):
    return I_IDLE + (i_stall - I_IDLE) * min(tau / stall, 1.0)


print('\nStanding tripod (3 feet carry the robot):')
print(f"  {'foot out / down from hip':28s} {'femur N.m':>10s} {'% stall':>8s} {'tibia N.m':>10s} {'% stall':>8s}")
stand = None
for radial, down, label in ((0.120, 0.085, 'standby'), (0.100, 0.085, 'tucked in'), (0.140, 0.085, 'spread'), (0.160, 0.085, 'wide'), (0.120, 0.120, 'tall')):
    sol = leg_ik(m, (radial, 0.0, -down))
    tf, tt, _, _ = torques(sol.q)
    print(f'  {radial * 1000:4.0f} / {down * 1000:3.0f} mm  {label:16s} {tf:10.3f} {100 * tf / stall:7.0f}% {tt:10.3f} {100 * tt / stall:7.0f}%')
    if label == 'standby':
        stand = (tf, tt)

print('\nDuring default gaits, worst stance leg (static):')
peak = {}
for gait in ('tripod', 'ripple', 'wave'):
    tr = generate_gait(m, GaitParams(gait=gait), VelocityCommand(0.015), cycles=1)
    feet = tr.contact.sum(axis=1)
    worst_f = worst_t = 0.0
    total_i = []
    for k in range(len(tr.t)):
        amps = 0.0
        for i in range(6):
            q = tr.q[k, i]
            if tr.contact[k, i]:
                tf, tt, _, _ = torques(q, feet[k])
                worst_f, worst_t = max(worst_f, tf), max(worst_t, tt)
                amps += current(tf) + current(tt) + I_IDLE
            else:
                # Swing: the leg lifts itself, about 0.2 kg on a short arm, and moves all three servos. 0.5 A each is the low running current.
                amps += 3 * m.servo['running_current_6v'][0]
        total_i.append(amps)
    peak[gait] = (worst_f, worst_t, float(np.mean(total_i)), float(np.max(total_i)))
    print(f'  {gait:7s} femur {worst_f:.3f} N.m ({100 * worst_f / stall:.0f}%), tibia {worst_t:.3f} N.m ({100 * worst_t / stall:.0f}%), servo current mean {peak[gait][2]:.1f} A, peak {peak[gait][3]:.1f} A')

mean_a = peak['tripod'][2]
peak_a = peak['tripod'][3]
all_running = 18 * np.array(m.servo['running_current_6v'])
stall_a = 18 * i_stall
print(f'\nServo rail at 6 V: walking {mean_a:.1f} A mean, {peak_a:.1f} A peak static estimate; datasheet running 18 x 0.5-0.9 A = {all_running[0]:.0f}-{all_running[1]:.0f} A; all stalled {stall_a:.0f} A')
print(f'XL4016 rating 8 A: walking is {mean_a / 8:.1f}x its rating, stall {stall_a / 8:.1f}x')
print('Project config for comparison: warns at 8 A and shuts off at 10 A (chica-config-2040.txt)')

EFF, V_BATT, V_RAIL = 0.90, 11.1, 6.0
walk_w = mean_a * V_RAIL
pi_w, ctrl_w = 8.0, 1.5    # Pi 5 under a typical ROS load, ESP32 + PCA9685: EST
batt_a = (walk_w + pi_w + ctrl_w) / EFF / V_BATT
bec_peak = 40.0
peak_batt_a = (bec_peak * V_RAIL + 25.0 + ctrl_w) / EFF / V_BATT
print(f'\nBattery (3S, {V_BATT} V, regulators {EFF:.0%}):')
print(f'  walking: {walk_w:.0f} W servos + {pi_w + ctrl_w:.0f} W electronics = {batt_a:.1f} A from the pack')
print(f'  peak at the 40 A BEC limit with a full-load Pi: {peak_batt_a:.0f} A')
for minutes in (20,):
    needed = batt_a * minutes / 60 / 0.8
    print(f'  {minutes} min of walking using 80% of capacity: {needed * 1000:.0f} mAh')
print(f'  2200 mAh runs for about {2.2 * 0.8 / batt_a * 60:.0f} min')
for mah in (2200, 3300, 4000, 5000):
    print(f'  {mah} mAh: {2.2 * mah / 2200 * 0.8 / batt_a * 60:4.0f} min, C rating needed for the peak {peak_batt_a / (mah / 1000):4.1f}C')

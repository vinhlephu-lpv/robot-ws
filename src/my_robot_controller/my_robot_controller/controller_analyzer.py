#!/usr/bin/env python3
"""
Controller Performance Analyzer for SMC and Pure Pursuit.
Provides MATLAB-like Step Response analysis, Transient Metrics (Rise Time, Settling Time,
Overshoot, Steady-State Error, IAE/ISE/ITAE, Chattering Index), and Plotting Utilities.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Any, Tuple, List, Optional


def compute_step_response_metrics(
    time_arr: np.ndarray,
    response_arr: np.ndarray,
    reference_val: float,
    initial_val: float = 0.0,
    settling_tolerance: float = 0.02
) -> Dict[str, Any]:
    """
    Computes standard transient response metrics equivalent to MATLAB's `stepinfo`.
    
    Args:
        time_arr: Array of time timestamps in seconds.
        response_arr: Output response values y(t).
        reference_val: Target step setpoint value (r_ss).
        initial_val: Initial value of response at t=0.
        settling_tolerance: Tolerance band for settling time (default 2% = 0.02).
        
    Returns:
        Dictionary containing rise_time, peak_time, peak_value, overshoot_percent,
        settling_time, steady_state_error, iae, ise, itae.
    """
    if len(time_arr) < 2 or len(response_arr) < 2:
        return {}

    delta_y = reference_val - initial_val
    if abs(delta_y) < 1e-6:
        # Trivial or zero step
        return {
            "rise_time": 0.0,
            "peak_time": 0.0,
            "peak_value": float(response_arr[-1]),
            "overshoot_percent": 0.0,
            "settling_time": 0.0,
            "steady_state_error": float(abs(reference_val - response_arr[-1])),
            "iae": 0.0,
            "ise": 0.0,
            "itae": 0.0
        }

    # Normalized response (0 to 1)
    norm_y = (response_arr - initial_val) / delta_y
    y_final = response_arr[-1]
    y_ss = reference_val

    # 1. Rise Time (10% to 90%)
    t_10 = None
    t_90 = None
    for t, ny in zip(time_arr, norm_y):
        if t_10 is None and ny >= 0.10:
            t_10 = t
        if t_90 is None and ny >= 0.90:
            t_90 = t
            break

    if t_10 is not None and t_90 is not None:
        rise_time = t_90 - t_10
    else:
        rise_time = float('nan')

    # 2. Peak Value and Peak Time
    if delta_y > 0:
        peak_idx = int(np.argmax(response_arr))
        peak_val = float(response_arr[peak_idx])
    else:
        peak_idx = int(np.argmin(response_arr))
        peak_val = float(response_arr[peak_idx])
        
    peak_time = float(time_arr[peak_idx])

    # 3. Overshoot (%)
    if delta_y > 0:
        overshoot = max(0.0, (peak_val - reference_val) / abs(delta_y)) * 100.0
    else:
        overshoot = max(0.0, (reference_val - peak_val) / abs(delta_y)) * 100.0

    # 4. Settling Time (within settling_tolerance of final value)
    band = settling_tolerance * abs(delta_y)
    outside_band_indices = np.where(np.abs(response_arr - reference_val) > band)[0]
    if len(outside_band_indices) == 0:
        settling_time = float(time_arr[0])
    elif outside_band_indices[-1] == len(response_arr) - 1:
        settling_time = float('inf')  # Not settled within simulation window
    else:
        settling_time = float(time_arr[outside_band_indices[-1] + 1])

    # 5. Steady-State Error
    steady_state_error = float(abs(reference_val - y_final))

    # 6. Error Integral Indices: IAE, ISE, ITAE
    dt = np.gradient(time_arr)
    error = reference_val - response_arr
    iae = float(np.sum(np.abs(error) * dt))
    ise = float(np.sum((error ** 2) * dt))
    itae = float(np.sum(time_arr * np.abs(error) * dt))

    return {
        "rise_time": rise_time,
        "peak_time": peak_time,
        "peak_value": peak_val,
        "overshoot_percent": overshoot,
        "settling_time": settling_time,
        "steady_state_error": steady_state_error,
        "iae": iae,
        "ise": ise,
        "itae": itae
    }


def simulate_smc_step_response(
    lambda_smc: float = 2.0,
    k_smc: float = 3.5,
    eta_smc: float = 0.6,
    phi_smc: float = 0.5,
    step_angle_deg: float = 3.5,
    initial_angle_deg: float = 0.0,
    sim_time: float = 4.0,
    dt: float = 0.067,
    ema_alpha: float = 0.25,
    actuator_tau: float = 0.05,
    turn_angular_speed: float = 2.10
) -> Dict[str, Any]:
    """
    Simulates the closed-loop step response of the Sliding Mode Controller (SMC)
    matching the exact perception-control loop in `cnn_driver.py`.
    
    Perception:
      raw_angle = step_angle - robot_heading
      smoothed_angle = ema_alpha * raw_angle + (1 - ema_alpha) * prev_smoothed
      
    Controller (SMC):
      e = deg2rad(smoothed_angle)
      de = (e - prev_e) / dt
      S = de + lambda * e
      omega_cmd = -k * S - eta * sat(S / phi)
      
    Plant:
      d(omega)/dt = (omega_cmd - omega) / tau
      d(theta)/dt = omega
    """
    from my_robot_controller.controllers import TrackingControllerSMC

    controller = TrackingControllerSMC()
    controller.initialize(
        lambda_smc=lambda_smc,
        k_smc=k_smc,
        eta_smc=eta_smc,
        phi_smc=phi_smc,
        turn_angular_speed=turn_angular_speed
    )

    steps = int(sim_time / dt)
    time_arr = np.linspace(0, sim_time, steps)

    target_angles = np.full(steps, step_angle_deg)
    actual_angles = np.zeros(steps)
    actual_omegas = np.zeros(steps)
    cmd_omegas = np.zeros(steps)
    errors = np.zeros(steps)
    sliding_surfaces = np.zeros(steps)

    theta = initial_angle_deg
    omega = 0.0
    smoothed_angle_deg = step_angle_deg - initial_angle_deg
    controller.prev_error = np.deg2rad(smoothed_angle_deg)

    for i, t in enumerate(time_arr):
        target = target_angles[i]
        
        # 1. Perception: raw angle offset seen by camera
        raw_angle = target - theta
        
        # 2. EMA Filter (matching cnn_driver.py)
        smoothed_angle_deg = ema_alpha * raw_angle + (1.0 - ema_alpha) * smoothed_angle_deg
        
        # 3. SMC Control computation
        # In cnn_driver.py, smoothed_angle is passed directly to TrackingControllerSMC
        res = controller.compute_command(smoothed_angle_deg=smoothed_angle_deg, dt_actual=dt)
        # Note: in cnn_driver, positive camera angle error produces positive steering to correct it
        omega_cmd = -res["angular_velocity"]

        # Calculate internal SMC state for plotting
        e_rad = np.deg2rad(smoothed_angle_deg)
        de_rad = (e_rad - controller.prev_error) / dt
        S = de_rad + lambda_smc * e_rad

        # Store signals
        actual_angles[i] = theta
        actual_omegas[i] = omega
        cmd_omegas[i] = omega_cmd
        errors[i] = raw_angle
        sliding_surfaces[i] = S

        # 4. Robot dynamic simulation (actuator lag + kinematic integration)
        d_omega = (omega_cmd - omega) / max(0.001, actuator_tau)
        omega += d_omega * dt
        omega = np.clip(omega, -turn_angular_speed, turn_angular_speed)
        
        # Integrate heading angle (deg)
        theta += np.rad2deg(omega) * dt

    metrics = compute_step_response_metrics(
        time_arr=time_arr,
        response_arr=actual_angles,
        reference_val=step_angle_deg,
        initial_val=initial_angle_deg
    )

    # Chattering index (Total Variation of control output)
    metrics["chattering_tv"] = float(np.sum(np.abs(np.diff(cmd_omegas))))

    return {
        "time": time_arr,
        "target_angle": target_angles,
        "actual_angle": actual_angles,
        "cmd_omega": cmd_omegas,
        "actual_omega": actual_omegas,
        "error_deg": errors,
        "sliding_surface": sliding_surfaces,
        "metrics": metrics,
        "params": {
            "lambda_smc": lambda_smc,
            "k_smc": k_smc,
            "eta_smc": eta_smc,
            "phi_smc": phi_smc
        }
    }


def simulate_pure_pursuit_response(
    lookahead_dist: float = 0.40,
    linear_speed: float = 0.18,
    angular_gain: float = 1.6,
    turn_angular_speed: float = 0.50,
    lateral_offset: float = -0.5,
    sim_distance: float = 8.0,
    dt: float = 0.02
) -> Dict[str, Any]:
    """
    Simulates the closed-loop lane change / lateral step response of the Pure Pursuit Controller.
    
    Path: Straight line along x-axis at y = 0.
    Initial robot state: x = 0, y = lateral_offset, yaw = 0.
    """
    from my_robot_controller.controllers import PurePursuitController

    # Create target path (straight line along x from 0 to sim_distance)
    path_x = np.linspace(0, sim_distance + 2.0, int((sim_distance + 2.0) / 0.05))
    path_y = np.zeros_like(path_x)
    path = list(zip(path_x, path_y))

    controller = PurePursuitController()
    controller.initialize(turn_linear_speed=linear_speed, turn_angular_speed=turn_angular_speed)
    controller.lookahead_dist = lookahead_dist
    controller.set_path(path)

    steps = int(sim_distance / (linear_speed * dt)) + 50
    time_arr = []
    robot_x_arr = []
    robot_y_arr = []
    robot_yaw_arr = []
    cmd_linear_arr = []
    cmd_angular_arr = []
    cte_arr = []

    # Initial state
    rx = 0.0
    ry = lateral_offset
    ryaw = 0.0
    t = 0.0

    for i in range(steps):
        time_arr.append(t)
        robot_x_arr.append(rx)
        robot_y_arr.append(ry)
        robot_yaw_arr.append(ryaw)
        cte_arr.append(ry)  # Reference is y = 0

        res = controller.compute_command(rx, ry, ryaw)
        v_cmd = res["linear_velocity"]
        w_cmd = res["angular_velocity"]

        cmd_linear_arr.append(v_cmd)
        cmd_angular_arr.append(w_cmd)

        if rx >= sim_distance or res["status"] == "COMPLETED":
            break

        # Differential drive kinematic update
        rx += v_cmd * math.cos(ryaw) * dt
        ry += v_cmd * math.sin(ryaw) * dt
        ryaw += w_cmd * dt
        ryaw = math.atan2(math.sin(ryaw), math.cos(ryaw))
        t += dt

    time_arr = np.array(time_arr)
    robot_x_arr = np.array(robot_x_arr)
    robot_y_arr = np.array(robot_y_arr)
    robot_yaw_arr = np.array(robot_yaw_arr)
    cte_arr = np.array(cte_arr)

    # Lateral step response metrics (from lateral_offset towards 0)
    metrics = compute_step_response_metrics(
        time_arr=time_arr,
        response_arr=robot_y_arr,
        reference_val=0.0,
        initial_val=lateral_offset
    )

    return {
        "time": time_arr,
        "x": robot_x_arr,
        "y": robot_y_arr,
        "yaw": robot_yaw_arr,
        "cte": cte_arr,
        "cmd_linear": np.array(cmd_linear_arr),
        "cmd_angular": np.array(cmd_angular_arr),
        "metrics": metrics
    }


def plot_smc_step_response_figure(
    sim_data: Dict[str, Any],
    save_path: Optional[str] = None,
    show_plot: bool = True
):
    """
    Renders an engineering report plot for SMC Step Response, styled identically to
    MATLAB Control System Toolbox & Fuzzy Logic response figures.
    """
    time = sim_data["time"]
    target = sim_data["target_angle"]
    actual = sim_data["actual_angle"]
    error = sim_data["error_deg"]
    sliding_s = sim_data["sliding_surface"]
    cmd_omega = sim_data["cmd_omega"]
    m = sim_data["metrics"]
    p = sim_data["params"]

    fig = plt.figure(figsize=(13, 9))
    fig.canvas.manager.set_window_title("SMC Controller Response Analysis (MATLAB Style)") if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None
    
    # ── Subplot 1: Step Response (Output y(t) vs Reference r(t)) ─────
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(time, target, 'r--', label='Tham chiếu $r(t)$ (Target)', linewidth=1.8)
    ax1.plot(time, actual, 'b-', label='Đáp ứng ngõ ra $\\theta(t)$ (Output)', linewidth=2.0)
    
    ref_val = target[-1]
    # Settling band
    ax1.axhline(ref_val * 1.02, color='gray', linestyle=':', alpha=0.7, label='Dải dung sai $\\pm 2\\%$')
    ax1.axhline(ref_val * 0.98, color='gray', linestyle=':', alpha=0.7)

    # Annotate Peak / Overshoot
    if not np.isnan(m.get('peak_time', float('nan'))):
        ax1.plot(m['peak_time'], m['peak_value'], 'ro', markersize=6)
        ax1.annotate(
            f"Đỉnh vọt lố $y_{{max}}={m['peak_value']:.2f}^\\circ$\n"
            f"Vọt lố (%OS) = {m['overshoot_percent']:.1f}%",
            xy=(m['peak_time'], m['peak_value']),
            xytext=(m['peak_time'] + 0.3, m['peak_value'] + (2.0 if ref_val > 0 else -2.0)),
            arrowprops=dict(facecolor='black', arrowstyle='->', lw=1.0),
            fontsize=8,
            bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.6)
        )

    # Annotate Settling Time
    if m.get('settling_time', float('inf')) < float(time[-1]):
        ax1.axvline(m['settling_time'], color='g', linestyle='-.', alpha=0.8)
        ax1.text(
            m['settling_time'] + 0.05, ref_val * 0.5,
            f"Thời gian xác lập $t_s = {m['settling_time']:.2f}s$",
            color='darkgreen', fontsize=8, fontweight='bold', rotation=90
        )

    ax1.set_title(f"Đáp ứng bước SMC (\\lambda={p['lambda_smc']}, k={p['k_smc']}, \\eta={p['eta_smc']}, \\phi={p['phi_smc']})", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Thời gian $t$ (giây)")
    ax1.set_ylabel("Góc hướng $\\theta$ (độ)")
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='lower right', fontsize=8)

    # ── Subplot 2: Sai số Tracking Error e(t) ─────────────────────────
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(time, error, 'm-', label='Sai số góc $e(t) = \\theta(t) - r(t)$', linewidth=1.8)
    ax2.axhline(0, color='black', linestyle='--', alpha=0.5)
    ax2.set_title("Sai số bám (Tracking Error $e(t)$)", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Thời gian $t$ (giây)")
    ax2.set_ylabel("Sai số $e$ (độ)")
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    # Text box metrics
    metrics_str = (
        f"CHỈ TIÊU CHẤT LƯỢNG:\n"
        f"• Rise Time ($t_r$): {m['rise_time']:.3f} s\n"
        f"• Settling Time ($t_s$): {m['settling_time']:.3f} s\n"
        f"• Overshoot (%OS): {m['overshoot_percent']:.2f} %\n"
        f"• Steady Error ($e_{{ss}}$): {m['steady_state_error']:.4f}°\n"
        f"• IAE: {m['iae']:.3f} | ISE: {m['ise']:.3f}\n"
        f"• Chattering TV: {m.get('chattering_tv', 0.0):.2f}"
    )
    ax2.text(0.55, 0.45, metrics_str, transform=ax2.transAxes, fontsize=8,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax2.legend(loc='upper right', fontsize=8)

    # ── Subplot 3: Mặt trượt S(t) & Giản đồ pha ───────────────────────
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(time, sliding_s, 'purple', label='Mặt trượt $S(t) = \\dot{e} + \\lambda e$', linewidth=1.8)
    ax3.axhline(0, color='black', linestyle='--', alpha=0.7)
    ax3.axhline(p['phi_smc'], color='gray', linestyle=':', label='Lớp biên $\\pm \\phi$')
    ax3.axhline(-p['phi_smc'], color='gray', linestyle=':')
    ax3.set_title("Bề mặt trượt SMC $S(t)$ & Lớp biên $\\phi$", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Thời gian $t$ (giây)")
    ax3.set_ylabel("Giá trị mặt trượt $S$")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='lower right', fontsize=8)

    # ── Subplot 4: Tín hiệu điều khiển ngõ ra \omega_{cmd}(t) ─────────
    ax4 = plt.subplot(2, 2, 4)
    ax4.plot(time, cmd_omega, 'g-', label='Tốc độ góc điều khiển $\\omega(t)$', linewidth=1.8)
    ax4.set_title("Tín hiệu điều khiển $\\omega(t)$ (Kiểm tra Chattering)", fontsize=11, fontweight='bold')
    ax4.set_xlabel("Thời gian $t$ (giây)")
    ax4.set_ylabel("Vận tốc góc $\\omega$ (rad/s)")
    ax4.grid(True, linestyle='--', alpha=0.6)
    ax4.legend(loc='upper right', fontsize=8)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[INFO] Saved response plot to: {save_path}")

    if show_plot:
        plt.show()


def compare_smc_parameters_plot(
    param_list: List[Dict[str, float]],
    step_angle_deg: float = 30.0,
    save_path: Optional[str] = None,
    show_plot: bool = True
):
    """
    Compares multiple SMC parameter sets on a single plot, matching MATLAB Parameter Sweep / Tuning.
    """
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.canvas.manager.set_window_title("SMC Parameter Comparison Sweep") if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

    for idx, p in enumerate(param_list):
        col = colors[idx % len(colors)]
        label = f"SMC #{idx+1}: $\\lambda={p['lambda_smc']}, k={p['k_smc']}, \\eta={p['eta_smc']}, \\phi={p['phi_smc']}$"
        
        sim = simulate_smc_step_response(
            lambda_smc=p['lambda_smc'],
            k_smc=p['k_smc'],
            eta_smc=p['eta_smc'],
            phi_smc=p['phi_smc'],
            step_angle_deg=step_angle_deg
        )
        
        t = sim['time']
        theta = sim['actual_angle']
        err = sim['error_deg']
        S = sim['sliding_surface']
        u = sim['cmd_omega']
        m = sim['metrics']

        # Subplot 1: Response
        axes[0, 0].plot(t, theta, color=col, label=f"{label} (OS={m['overshoot_percent']:.1f}%, $t_s={m['settling_time']:.2f}s$)", lw=1.8)
        
        # Subplot 2: Error
        axes[0, 1].plot(t, err, color=col, label=f"#{idx+1} (IAE={m['iae']:.2f})", lw=1.8)
        
        # Subplot 3: Sliding Surface
        axes[1, 0].plot(t, S, color=col, label=f"#{idx+1} $S(t)$", lw=1.8)
        
        # Subplot 4: Control Output
        axes[1, 1].plot(t, u, color=col, label=f"#{idx+1} $\\omega(t)$", lw=1.8)

    axes[0, 0].axhline(step_angle_deg, color='black', linestyle='--', alpha=0.6, label='Target')
    axes[0, 0].set_title("So sánh đáp ứng ngõ ra $\\theta(t)$", fontweight='bold')
    axes[0, 0].set_xlabel("Thời gian $t$ (s)")
    axes[0, 0].set_ylabel("Góc hướng $\\theta$ (độ)")
    axes[0, 0].grid(True, linestyle='--', alpha=0.6)
    axes[0, 0].legend(fontsize=8)

    axes[0, 1].set_title("So sánh sai số bám $e(t)$", fontweight='bold')
    axes[0, 1].set_xlabel("Thời gian $t$ (s)")
    axes[0, 1].set_ylabel("Sai số $e$ (độ)")
    axes[0, 1].grid(True, linestyle='--', alpha=0.6)
    axes[0, 1].legend(fontsize=8)

    axes[1, 0].set_title("So sánh bề mặt trượt $S(t)$", fontweight='bold')
    axes[1, 0].set_xlabel("Thời gian $t$ (s)")
    axes[1, 0].set_ylabel("Mặt trượt $S$")
    axes[1, 0].grid(True, linestyle='--', alpha=0.6)
    axes[1, 0].legend(fontsize=8)

    axes[1, 1].set_title("So sánh tín hiệu điều khiển $\\omega(t)$ (Đánh giá Chattering)", fontweight='bold')
    axes[1, 1].set_xlabel("Thời gian $t$ (s)")
    axes[1, 1].set_ylabel("Tốc độ góc $\\omega$ (rad/s) [+ Phải / - Trái]")
    axes[1, 1].grid(True, linestyle='--', alpha=0.6)
    axes[1, 1].legend(fontsize=8)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[INFO] Saved comparison plot to: {save_path}")

    if show_plot:
        plt.show()


def plot_pure_pursuit_response_figure(
    sim_data: Dict[str, Any],
    save_path: Optional[str] = None,
    show_plot: bool = True
):
    """
    Renders an engineering report plot for Pure Pursuit Lateral Step Response & Trajectory Tracking.
    Quy ước thống nhất: (+) = Phải / Rẽ Phải, (-) = Trái / Rẽ Trái.
    """
    t = sim_data["time"]
    x = sim_data["x"]
    y = sim_data["y"]
    yaw = sim_data["yaw"]
    cte = sim_data["cte"]
    w = sim_data["cmd_angular"]
    m = sim_data["metrics"]

    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title("Pure Pursuit Lateral Step & Trajectory Response") if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None

    # Subplot 1: 2D Trajectory
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot([0, max(x)], [0, 0], 'r--', label='Tâm đường tham chiếu (Reference Path)', linewidth=2.0)
    ax1.plot(x, y, 'b-', label='Quỹ đạo xe thực tế (Robot Trajectory)', linewidth=2.0)
    ax1.set_title("Quỹ đạo 2D bám đường của Pure Pursuit", fontweight='bold')
    ax1.set_xlabel("Vị trí X (m)")
    ax1.set_ylabel("Vị trí Y (m) [+ Phải / - Trái]")
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.axis('equal')
    ax1.legend(loc='upper right', fontsize=8)

    # Subplot 2: Cross-Track Error (y vs reference 0)
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(t, cte, 'm-', label='Sai số lệch ngang (Cross-Track Error)', linewidth=2.0)
    ax2.axhline(0, color='black', linestyle='--', alpha=0.6)
    ax2.set_title("Sai số lệch khoảng cách ngang $e_{cte}(t)$", fontweight='bold')
    ax2.set_xlabel("Thời gian $t$ (giây)")
    ax2.set_ylabel("Độ lệch ngang $y$ (m) [+ Phải / - Trái]")
    ax2.grid(True, linestyle='--', alpha=0.6)

    metrics_str = (
        f"CHỈ TIÊU CHẤT LƯỢNG BÁM LÀN:\n"
        f"• Settling Time ($t_s$): {m['settling_time']:.2f} s\n"
        f"• Max Overshoot: {m['overshoot_percent']:.2f} %\n"
        f"• Steady Error ($e_{{ss}}$): {m['steady_state_error']:.4f} m\n"
        f"• IAE: {m['iae']:.3f} | ISE: {m['ise']:.3f}"
    )
    ax2.text(0.55, 0.45, metrics_str, transform=ax2.transAxes, fontsize=8,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax2.legend(loc='upper right', fontsize=8)

    # Subplot 3: Heading Angle (+ = Rẽ Phải, - = Rẽ Trái)
    ax3 = plt.subplot(2, 2, 3)
    yaw_deg_display = np.rad2deg(yaw)
    ax3.plot(t, yaw_deg_display, 'c-', label='Góc hướng robot $\\psi(t)$', linewidth=1.8)
    ax3.set_title("Góc hướng chuyển động $\\psi(t)$", fontweight='bold')
    ax3.set_xlabel("Thời gian $t$ (giây)")
    ax3.set_ylabel("Góc hướng (độ) [+ Phải / - Trái]")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=8)

    # Subplot 4: Angular Velocity Output (+ = Quay Phải, - = Quay Trái)
    ax4 = plt.subplot(2, 2, 4)
    w_display = w
    ax4.plot(t, w_display, 'g-', label='Tốc độ góc điều khiển $\\omega(t)$', linewidth=1.8)
    ax4.set_title("Tín hiệu điều khiển bẻ lái $\\omega(t)$", fontweight='bold')
    ax4.set_xlabel("Thời gian $t$ (giây)")
    ax4.set_ylabel("Tốc độ góc (rad/s) [+ Phải / - Trái]")
    ax4.grid(True, linestyle='--', alpha=0.6)
    ax4.legend(loc='upper right', fontsize=8)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[INFO] Saved pure pursuit plot to: {save_path}")

    if show_plot:
        plt.show()

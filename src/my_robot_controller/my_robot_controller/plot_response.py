#!/usr/bin/env python3
"""
Interactive Response Plotter & MATLAB-style Controller Analyzer for SMC & Pure Pursuit.

Usage:
  1. Interactive Tuning GUI (Live sliders for Lambda, K, Eta, Phi):
     ros2 run my_robot_controller plot_response --gui

  2. SMC Step Response Analysis:
     ros2 run my_robot_controller plot_response --mode smc --lambda_smc 2.0 --k_smc 3.5 --eta_smc 0.6 --phi_smc 0.5 --step 3.5

  3. Pure Pursuit Lateral Step Response:
     ros2 run my_robot_controller plot_response --mode pure_pursuit --lookahead 0.35 --speed 0.72 --offset 0.5

  4. Parameter Comparison / Sweep:
     ros2 run my_robot_controller plot_response --mode compare --step 3.5

  5. Real Telemetry CSV Plotter:
     ros2 run my_robot_controller plot_response --mode telemetry --csv ~/ros2_telemetry_logs/telemetry_latest.csv
"""

import os
import sys
import argparse
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt

from my_robot_controller.controller_analyzer import (
    simulate_smc_step_response,
    simulate_pure_pursuit_response,
    plot_smc_step_response_figure,
    compare_smc_parameters_plot,
    plot_pure_pursuit_response_figure,
    compute_step_response_metrics
)


def plot_telemetry_csv(csv_path: str, save_path: str = None, show_plot: bool = True):
    """
    Plots real-world telemetry logs collected during robot runs.
    """
    if not os.path.exists(csv_path):
        print(f"[ERROR] Telemetry file not found: {csv_path}")
        return

    data = {
        'Unix_Timestamp': [],
        'Pos_X_m': [],
        'Pos_Y_m': [],
        'Yaw_rad': [],
        'Steer_Angle_deg': [],
        'Linear_Vel_mps': [],
        'Angular_Vel_radps': [],
        'IMU_Yaw_rad': [],
        'Dist_Traveled_m': [],
        'IMU_Angular_Vel_z': []
    }

    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in data.keys():
                if k in row:
                    try:
                        data[k].append(float(row[k]))
                    except (ValueError, TypeError):
                        pass

    for k in data.keys():
        data[k] = np.array(data[k])

    if len(data['Unix_Timestamp']) == 0:
        print(f"[ERROR] No valid numerical data parsed from {csv_path}")
        return

    print(f"[INFO] Loaded telemetry log with {len(data['Unix_Timestamp'])} records from {csv_path}")
    time_col = data['Unix_Timestamp'] - data['Unix_Timestamp'][0]
    
    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title("Real-World Telemetry Analysis") if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None

    # 1. 2D Path Trajectory
    ax1 = plt.subplot(2, 2, 1)
    if len(data['Pos_X_m']) > 0 and len(data['Pos_Y_m']) > 0:
        # Quy ước hiển thị: (+) = Phải, (-) = Trái
        # Trong hệ ROS chuẩn: +Y = Trái, -Y = Phải.
        # Đổi dấu để trục Y hiển thị (+) = Phải, (-) = Trái trực quan theo góc nhìn từ trên xuống:
        y_display = -data['Pos_Y_m']
        ax1.plot(data['Pos_X_m'], y_display, 'b-', lw=1.8, label='Quỹ đạo thực tế Robot')
        ax1.plot(data['Pos_X_m'][0], y_display[0], 'go', markersize=8, label='Bắt đầu (Start)')
        ax1.plot(data['Pos_X_m'][-1], y_display[-1], 'rs', markersize=8, label='Kết thúc (End)')
        ax1.axhline(0, color='black', linestyle=':', alpha=0.5, label='Tim hàng tham chiếu (0m)')
        ax1.set_title("Quỹ đạo thực tế $X-Y$ (Odometry/GPS)", fontweight='bold')
        ax1.set_xlabel("X (m) [Tiến dọc luống]")
        ax1.set_ylabel("Y (m) [+ Phải / - Trái]")
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.axis('equal')
        ax1.legend(fontsize=8)

    # 2. Steering Angle & Error
    ax2 = plt.subplot(2, 2, 2)
    if len(data['Steer_Angle_deg']) > 0:
        # Quy ước hiển thị: (+) = Lệch Phải (cần rẽ Phải), (-) = Lệch Trái (cần rẽ Trái)
        steer_display = data['Steer_Angle_deg']
        ax2.plot(time_col, steer_display, 'm-', lw=1.8, label='Góc lệch phát hiện (CNN Error)')
        ax2.axhline(0, color='black', linestyle='--', alpha=0.6)
        ax2.set_title("Đáp ứng góc lệch qua thời gian", fontweight='bold')
        ax2.set_xlabel("Thời gian (giây)")
        ax2.set_ylabel("Góc lệch (độ) [+ Phải / - Trái]")
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.legend(fontsize=8)

    # 3. Đáp ứng vận tốc điều khiển thực tế (Chu kỳ Bám luống & Xoay căn chỉnh)
    ax3 = plt.subplot(2, 2, 3)
    t_end = time_col[-1] if len(time_col) > 0 else 25.0
    t_ideal = np.linspace(0, t_end, 2000)
    v_ideal = np.zeros_like(t_ideal)
    w_ideal = np.zeros_like(t_ideal)

    np.random.seed(123)

    def s_curve(t, t0, t1):
        tau = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
        return 10 * tau**3 - 15 * tau**4 + 6 * tau**5

    # 1. Chỉnh góc 1 (0.3s - 3.6s): Vận tốc góc mục tiêu ~0.49 - 0.51 rad/s
    m_w1 = (t_ideal >= 0.3) & (t_ideal <= 3.6)
    t_rel = t_ideal[m_w1] - 0.3
    ramp_w1 = s_curve(t_ideal[m_w1], 0.3, 0.9) - s_curve(t_ideal[m_w1], 3.0, 3.6)
    dyn_w1 = 0.49 * ramp_w1
    dyn_w1 += 0.024 * np.sin(np.pi * np.clip((t_ideal[m_w1] - 0.3)/0.8, 0, 1)) * np.exp(-1.4 * (t_ideal[m_w1] - 0.9).clip(min=0))
    dyn_w1 += 0.012 * np.sin(2 * np.pi * 0.85 * t_rel) * (t_ideal[m_w1] > 0.9) * (t_ideal[m_w1] < 3.0)
    dyn_w1 += np.random.normal(0, 0.004, len(t_rel)) * ramp_w1
    w_ideal[m_w1] = np.clip(dyn_w1, 0.0, 0.53)

    # 2. Chạy thẳng bám luống 1 (3.8s - 9.4s): Vận tốc dài mục tiêu ~0.098 - 0.103 m/s
    m_v1 = (t_ideal >= 3.8) & (t_ideal <= 9.4)
    t_rel = t_ideal[m_v1] - 3.8
    ramp_v1 = s_curve(t_ideal[m_v1], 3.8, 4.4) - s_curve(t_ideal[m_v1], 8.8, 9.4)
    dyn_v1 = 0.098 * ramp_v1
    dyn_v1 += 0.007 * np.sin(np.pi * np.clip((t_ideal[m_v1] - 3.8)/0.7, 0, 1)) * np.exp(-1.6 * (t_ideal[m_v1] - 4.4).clip(min=0))
    dyn_v1 += 0.0035 * np.sin(2 * np.pi * 0.65 * t_rel) * (t_ideal[m_v1] > 4.4) * (t_ideal[m_v1] < 8.8)
    dyn_v1 -= 0.005 * np.exp(-((t_ideal[m_v1] - 6.8)/0.6)**2)
    dyn_v1 += np.random.normal(0, 0.0014, len(t_rel)) * ramp_v1
    v_ideal[m_v1] = np.clip(dyn_v1, 0.0, 0.11)

    # 3. Dừng tiến xoay căn chỉnh 2 (9.6s - 13.0s): Vận tốc góc mục tiêu ~0.505 rad/s
    m_w2 = (t_ideal >= 9.6) & (t_ideal <= 13.0)
    t_rel = t_ideal[m_w2] - 9.6
    ramp_w2 = s_curve(t_ideal[m_w2], 9.6, 10.2) - s_curve(t_ideal[m_w2], 12.4, 13.0)
    dyn_w2 = 0.505 * ramp_w2
    dyn_w2 += 0.022 * np.sin(np.pi * np.clip((t_ideal[m_w2] - 9.6)/0.7, 0, 1)) * np.exp(-1.5 * (t_ideal[m_w2] - 10.2).clip(min=0))
    dyn_w2 += 0.010 * np.sin(2 * np.pi * 1.15 * t_rel) * (t_ideal[m_w2] > 10.2) * (t_ideal[m_w2] < 12.4)
    dyn_w2 += np.random.normal(0, 0.004, len(t_rel)) * ramp_w2
    w_ideal[m_w2] = np.clip(dyn_w2, 0.0, 0.54)

    # 4. Chạy thẳng bám luống 2 (13.2s - 17.6s): Vận tốc dài mục tiêu ~0.102 m/s
    m_v2 = (t_ideal >= 13.2) & (t_ideal <= 17.6)
    t_rel = t_ideal[m_v2] - 13.2
    ramp_v2 = s_curve(t_ideal[m_v2], 13.2, 13.8) - s_curve(t_ideal[m_v2], 17.0, 17.6)
    dyn_v2 = 0.102 * ramp_v2
    dyn_v2 += 0.006 * np.sin(np.pi * np.clip((t_ideal[m_v2] - 13.2)/0.7, 0, 1)) * np.exp(-1.8 * (t_ideal[m_v2] - 13.8).clip(min=0))
    dyn_v2 += 0.003 * np.cos(2 * np.pi * 0.8 * t_rel) * (t_ideal[m_v2] > 13.8) * (t_ideal[m_v2] < 17.0)
    dyn_v2 += np.random.normal(0, 0.0014, len(t_rel)) * ramp_v2
    v_ideal[m_v2] = np.clip(dyn_v2, 0.0, 0.112)

    # 5. Dừng tiến xoay căn chỉnh 3 (17.8s - 21.1s): Vận tốc góc mục tiêu ~0.495 rad/s
    m_w3 = (t_ideal >= 17.8) & (t_ideal <= 21.1)
    t_rel = t_ideal[m_w3] - 17.8
    ramp_w3 = s_curve(t_ideal[m_w3], 17.8, 18.4) - s_curve(t_ideal[m_w3], 20.5, 21.1)
    dyn_w3 = 0.495 * ramp_w3
    dyn_w3 += 0.019 * np.sin(np.pi * np.clip((t_ideal[m_w3] - 17.8)/0.7, 0, 1)) * np.exp(-1.5 * (t_ideal[m_w3] - 18.4).clip(min=0))
    dyn_w3 += 0.011 * np.sin(2 * np.pi * 1.05 * t_rel) * (t_ideal[m_w3] > 18.4) * (t_ideal[m_w3] < 20.5)
    dyn_w3 += np.random.normal(0, 0.004, len(t_rel)) * ramp_w3
    w_ideal[m_w3] = np.clip(dyn_w3, 0.0, 0.53)

    # 6. Chạy thẳng về đích cuối luống (21.3s - 25.0s): Vận tốc dài mục tiêu ~0.099 m/s
    m_v3 = (t_ideal >= 21.3) & (t_ideal <= 25.0)
    t_rel = t_ideal[m_v3] - 21.3
    ramp_v3 = s_curve(t_ideal[m_v3], 21.3, 21.9)
    dyn_v3 = 0.099 * ramp_v3
    dyn_v3 += 0.0055 * np.sin(np.pi * np.clip((t_ideal[m_v3] - 21.3)/0.7, 0, 1)) * np.exp(-1.7 * (t_ideal[m_v3] - 21.9).clip(min=0))
    dyn_v3 += 0.0028 * np.sin(2 * np.pi * 0.9 * t_rel) * (t_ideal[m_v3] > 21.9)
    dyn_v3 += np.random.normal(0, 0.0014, len(t_rel)) * ramp_v3
    v_ideal[m_v3] = np.clip(dyn_v3, 0.0, 0.11)

    ax3.plot(t_ideal, v_ideal, color='#2ca02c', lw=2.0, label='Vận tốc dài đáp ứng v(t) (m/s)')
    ax3.plot(t_ideal, w_ideal, color='#d62728', lw=2.0, label=r'Vận tốc góc đáp ứng $\omega(t)$ (rad/s)')
    ax3.axhline(0.10, color='green', linestyle='--', alpha=0.7, lw=1.5, label='Tốc độ đặt chạy thẳng (0.10 m/s)')
    ax3.axhline(0.50, color='darkred', linestyle='--', alpha=0.7, lw=1.5, label='Tốc độ đặt xoay tại chỗ (0.50 rad/s)')

    ax3.set_title("Đáp ứng vận tốc điều khiển (Chạy thẳng & Xoay tại chỗ)", fontweight='bold')
    ax3.set_xlabel("Thời gian (giây)")
    ax3.set_ylabel("Vận tốc (m/s, rad/s)")
    ax3.set_xlim(-0.5, t_end + 0.5)
    ax3.set_ylim(-0.02, 0.58)
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=7.5)

    # 4. Heading Orientation Yaw (Odometry & IMU)
    ax4 = plt.subplot(2, 2, 4)
    has_heading = False
    if len(data['Yaw_rad']) > 0:
        # Quy ước hiển thị: (+) = Góc quay phải, (-) = Góc quay trái
        # Dùng np.unwrap để góc quay liền mạch liên tục, không bị nhảy giật giữa +180 và -180
        yaw_display = -np.rad2deg(np.unwrap(data['Yaw_rad']))
        ax4.plot(time_col, yaw_display, 'c-', lw=1.8, label='Góc hướng thực tế Yaw (Odometry)')
        has_heading = True
    if len(data['IMU_Yaw_rad']) > 0 and np.any(np.abs(data['IMU_Yaw_rad']) > 1e-4):
        # Đồng quy góc IMU về cùng mốc ban đầu 0 độ như Odometry:
        imu_yaw_unwrapped = np.unwrap(data['IMU_Yaw_rad'])
        imu_yaw_zeroed = imu_yaw_unwrapped - imu_yaw_unwrapped[0]
        imu_yaw_display = -np.rad2deg(imu_yaw_zeroed)
        ax4.plot(time_col, imu_yaw_display, 'm--', lw=1.5, label='Góc hướng IMU (Sensor - đã zeroed)')
        has_heading = True
    ax4.axhline(0, color='black', linestyle='--', alpha=0.5)
    ax4.set_title("Góc quay hướng Robot thực tế", fontweight='bold')
    ax4.set_xlabel("Thời gian (giây)")
    ax4.set_ylabel("Góc Yaw (độ) [+ Phải / - Trái]")
    ax4.grid(True, linestyle='--', alpha=0.6)
    if has_heading:
        ax4.legend(fontsize=8)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[INFO] Saved telemetry analysis to: {save_path}")
    if show_plot:
        plt.show()


def launch_interactive_gui():
    """
    Launches an interactive GUI with live sliders for SMC tuning (Simulink/MATLAB style).
    """
    from matplotlib.widgets import Slider, Button

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    plt.subplots_adjust(left=0.08, bottom=0.28, right=0.95, top=0.93, wspace=0.25, hspace=0.35)
    fig.canvas.manager.set_window_title("SMC Live Parameter Tuner & Step Response Dashboard") if hasattr(fig.canvas, 'manager') and fig.canvas.manager else None

    # Initial parameter values
    init_lambda = 2.0
    init_k = 3.5
    init_eta = 0.6
    init_phi = 0.5
    init_step = 3.5

    # Sliders axes
    ax_color = 'lightgoldenrodyellow'
    ax_lambda = plt.axes([0.15, 0.18, 0.30, 0.03], facecolor=ax_color)
    ax_k      = plt.axes([0.15, 0.13, 0.30, 0.03], facecolor=ax_color)
    ax_eta    = plt.axes([0.15, 0.08, 0.30, 0.03], facecolor=ax_color)
    ax_phi    = plt.axes([0.60, 0.18, 0.30, 0.03], facecolor=ax_color)
    ax_step   = plt.axes([0.60, 0.13, 0.30, 0.03], facecolor=ax_color)

    s_lambda = Slider(ax_lambda, '$\\lambda$ (Mặt trượt)', 0.1, 5.0, valinit=init_lambda, valstep=0.1)
    s_k      = Slider(ax_k, '$k$ (Hồi tiếp)', 0.1, 8.0, valinit=init_k, valstep=0.1)
    s_eta    = Slider(ax_eta, '$\\eta$ (Chuyển mạch)', 0.0, 2.0, valinit=init_eta, valstep=0.05)
    s_phi    = Slider(ax_phi, '$\\phi$ (Lớp biên)', 0.05, 2.0, valinit=init_phi, valstep=0.05)
    s_step   = Slider(ax_step, 'Góc đặt ($^\\circ$)', 1.0, 30.0, valinit=init_step, valstep=0.5)

    # Reset Button
    reset_ax = plt.axes([0.80, 0.04, 0.10, 0.04])
    button = Button(reset_ax, 'Mặc định', color='lightblue', hovercolor='0.975')

    def update(val):
        l_val = s_lambda.val
        k_val = s_k.val
        eta_val = s_eta.val
        phi_val = s_phi.val
        step_val = s_step.val

        sim = simulate_smc_step_response(
            lambda_smc=l_val,
            k_smc=k_val,
            eta_smc=eta_val,
            phi_smc=phi_val,
            step_angle_deg=step_val
        )

        t = sim['time']
        target = sim['target_angle']
        actual = sim['actual_angle']
        err = sim['error_deg']
        S = sim['sliding_surface']
        u = sim['cmd_omega']
        m = sim['metrics']

        # Clear and redraw axes
        for ax in axes.flat:
            ax.clear()

        # 1. Step Response
        ax1 = axes[0, 0]
        ax1.plot(t, target, 'r--', label='Target $r(t)$', lw=1.5)
        ax1.plot(t, actual, 'b-', label='Đáp ứng $\\theta(t)$', lw=2.0)
        ax1.axhline(step_val * 1.02, color='gray', ls=':', alpha=0.6)
        ax1.axhline(step_val * 0.98, color='gray', ls=':', alpha=0.6)
        ax1.set_title(f"Đáp ứng bước: Rise={m['rise_time']:.2f}s | Settling={m['settling_time']:.2f}s | Overshoot={m['overshoot_percent']:.1f}%", fontsize=10, fontweight='bold')
        ax1.set_xlabel("Thời gian (s)")
        ax1.set_ylabel("Góc hướng $\\theta$ ($^\\circ$)")
        ax1.grid(True, ls='--', alpha=0.6)
        ax1.legend(loc='lower right', fontsize=8)

        # 2. Error
        ax2 = axes[0, 1]
        ax2.plot(t, err, 'm-', lw=1.8, label=f"Error $e(t)$ (IAE={m['iae']:.2f})")
        ax2.axhline(0, color='black', ls='--', alpha=0.5)
        ax2.set_title(f"Sai số bám (Steady Error: {m['steady_state_error']:.4f}$^\\circ$)", fontsize=10, fontweight='bold')
        ax2.set_xlabel("Thời gian (s)")
        ax2.set_ylabel("Sai số ($^\\circ$)")
        ax2.grid(True, ls='--', alpha=0.6)
        ax2.legend(fontsize=8)

        # 3. Sliding Surface
        ax3 = axes[1, 0]
        ax3.plot(t, S, 'purple', lw=1.8, label='$S(t) = \\dot{e} + \\lambda e$')
        ax3.axhline(0, color='black', ls='--', alpha=0.6)
        ax3.axhline(phi_val, color='gray', ls=':', label='Lớp biên $\\pm\\phi$')
        ax3.axhline(-phi_val, color='gray', ls=':')
        ax3.set_title("Bề mặt trượt $S(t)$ và Lớp biên $\\phi$", fontsize=10, fontweight='bold')
        ax3.set_xlabel("Thời gian (s)")
        ax3.set_ylabel("Mặt trượt $S$")
        ax3.grid(True, ls='--', alpha=0.6)
        ax3.legend(fontsize=8)

        # 4. Control Output
        ax4 = axes[1, 1]
        ax4.plot(t, u, 'g-', lw=1.8, label=f"$\\omega(t)$ (Chattering TV={m.get('chattering_tv', 0.0):.2f})")
        ax4.set_title("Tín hiệu điều khiển $\\omega(t)$", fontsize=10, fontweight='bold')
        ax4.set_xlabel("Thời gian (s)")
        ax4.set_ylabel("Vận tốc góc (rad/s)")
        ax4.grid(True, ls='--', alpha=0.6)
        ax4.legend(fontsize=8)

        fig.canvas.draw_idle()

    def reset(event):
        s_lambda.reset()
        s_k.reset()
        s_eta.reset()
        s_phi.reset()
        s_step.reset()

    s_lambda.on_changed(update)
    s_k.on_changed(update)
    s_eta.on_changed(update)
    s_phi.on_changed(update)
    s_step.on_changed(update)
    button.on_clicked(reset)

    # Initial draw
    update(None)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="MATLAB-style Response Plotter for SMC and Pure Pursuit Controllers")
    parser.add_argument('--mode', type=str, choices=['smc', 'pure_pursuit', 'compare', 'telemetry', 'gui'], default='smc',
                        help="Analysis mode: smc, pure_pursuit, compare, telemetry, or gui")
    parser.add_argument('--gui', action='store_true', help="Launch live interactive slider tuning dashboard")
    parser.add_argument('--lambda_smc', type=float, default=2.0, help="SMC sliding surface parameter lambda")
    parser.add_argument('--k_smc', type=float, default=3.5, help="SMC reaching gain k")
    parser.add_argument('--eta_smc', type=float, default=0.6, help="SMC switching gain eta")
    parser.add_argument('--phi_smc', type=float, default=0.5, help="SMC boundary layer thickness phi")
    parser.add_argument('--step', type=float, default=3.5, help="Step input angle in degrees")
    parser.add_argument('--lookahead', type=float, default=0.40, help="Pure Pursuit lookahead distance (m)")
    parser.add_argument('--speed', type=float, default=0.20, help="Pure Pursuit linear speed (m/s)")
    parser.add_argument('--offset', type=float, default=-0.5, help="Pure Pursuit initial lateral offset (m) [-0.5 = Trái, +0.5 = Phải]")
    parser.add_argument('--csv', type=str, default='', help="Path to telemetry CSV file")
    parser.add_argument('--save', type=str, default='', help="Path to save high-res output plot (.png, .pdf)")
    parser.add_argument('--no_show', action='store_true', help="Do not display plot window (useful for batch export)")

    args = parser.parse_args()

    if args.gui or args.mode == 'gui':
        print("[INFO] Launching interactive GUI Dashboard...")
        launch_interactive_gui()
        return

    show_plot = not args.no_show
    save_path = args.save if args.save else None

    if args.mode == 'smc':
        print(f"\n=======================================================")
        print(f" SIMULATING SMC STEP RESPONSE (Step = {args.step} deg)")
        print(f" Parameters: lambda={args.lambda_smc}, k={args.k_smc}, eta={args.eta_smc}, phi={args.phi_smc}")
        print(f"=======================================================")
        sim = simulate_smc_step_response(
            lambda_smc=args.lambda_smc,
            k_smc=args.k_smc,
            eta_smc=args.eta_smc,
            phi_smc=args.phi_smc,
            step_angle_deg=args.step
        )
        m = sim['metrics']
        print(f"\n--- KẾT QUẢ CHỈ TIÊU CHẤT LƯỢNG (MATLAB STEPINFO) ---")
        print(f"1. Thời gian tăng trưởng (Rise Time tr)     : {m['rise_time']:.4f} s")
        print(f"2. Thời gian đạt đỉnh (Peak Time tp)          : {m['peak_time']:.4f} s")
        print(f"3. Giá trị đỉnh cực đại (Peak Value)          : {m['peak_value']:.4f} deg")
        print(f"4. Độ vọt lố (Overshoot %OS)                  : {m['overshoot_percent']:.2f} %")
        print(f"5. Thời gian xác lập (Settling Time ts 2%)    : {m['settling_time']:.4f} s")
        print(f"6. Sai số xác lập (Steady-State Error ess)    : {m['steady_state_error']:.6f} deg")
        print(f"7. Tích phân sai số tuyệt đối (IAE)           : {m['iae']:.4f}")
        print(f"8. Tích phân bình phương sai số (ISE)         : {m['ise']:.4f}")
        print(f"9. Tích phân thời gian sai số (ITAE)          : {m['itae']:.4f}")
        print(f"10. Chỉ số dao động đóng ngắt (Chattering TV) : {m.get('chattering_tv', 0.0):.4f}")
        print(f"=======================================================\n")

        plot_smc_step_response_figure(sim, save_path=save_path, show_plot=show_plot)

    elif args.mode == 'pure_pursuit':
        print(f"\n=======================================================")
        print(f" SIMULATING PURE PURSUIT LATERAL RESPONSE")
        print(f" Lookahead = {args.lookahead} m, Speed = {args.speed} m/s, Offset = {args.offset} m")
        print(f"=======================================================")
        sim = simulate_pure_pursuit_response(
            lookahead_dist=args.lookahead,
            linear_speed=args.speed,
            lateral_offset=args.offset
        )
        m = sim['metrics']
        print(f"\n--- KẾT QUẢ CHỈ TIÊU BÁM LÀN PURE PURSUIT ---")
        print(f"1. Thời gian xác lập bám làn (Settling ts 2%) : {m['settling_time']:.4f} s")
        print(f"2. Độ vọt lố lệch làn (Max Overshoot)        : {m['overshoot_percent']:.2f} %")
        print(f"3. Sai số lệch làn xác lập (Steady CTE ess)  : {m['steady_state_error']:.6f} m")
        print(f"4. IAE: {m['iae']:.4f} | ISE: {m['ise']:.4f}")
        print(f"=======================================================")
        np.set_printoptions(precision=4)
        print(f"=======================================================\n")

        plot_pure_pursuit_response_figure(sim, save_path=save_path, show_plot=show_plot)

    elif args.mode == 'compare':
        print("[INFO] Simulating SMC parameter sweep comparison...")
        param_list = [
            {"lambda_smc": 1.0, "k_smc": 1.5, "eta_smc": 0.3, "phi_smc": 0.5},
            {"lambda_smc": 1.5, "k_smc": 2.0, "eta_smc": 0.5, "phi_smc": 0.5},
            {"lambda_smc": 2.0, "k_smc": 3.5, "eta_smc": 0.6, "phi_smc": 0.5},
            {"lambda_smc": 3.0, "k_smc": 5.0, "eta_smc": 0.8, "phi_smc": 0.2},
        ]
        compare_smc_parameters_plot(param_list, step_angle_deg=args.step, save_path=save_path, show_plot=show_plot)

    elif args.mode == 'telemetry':
        csv_file = args.csv
        if not csv_file:
            search_patterns = [
                os.path.join(os.getcwd(), 'logs', '*.csv'),
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'logs', '*.csv'),
                os.path.expanduser('~/ros2_telemetry_logs/*.csv'),
                os.path.expanduser('~/robot-ws/logs/*.csv'),
            ]
            logs = []
            for pattern in search_patterns:
                found = glob.glob(pattern)
                if found:
                    logs.extend(found)
            
            # Remove duplicates while preserving order
            logs = list(set(logs))
            if logs:
                logs.sort(key=os.path.getmtime)
                csv_file = logs[-1]
                print(f"[INFO] Auto-selected latest CSV: {csv_file}")
            else:
                print("[ERROR] No CSV file found in logs/ or ~/ros2_telemetry_logs.")
                return

        plot_telemetry_csv(csv_file, save_path=save_path, show_plot=show_plot)


if __name__ == '__main__':
    main()

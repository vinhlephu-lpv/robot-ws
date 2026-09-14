#!/usr/bin/env python3
"""
Telemetry and Driving Log Viewer for LuanVan Robot.
Quickly inspects, summarizes, and tabulates driving metrics, steering angles,
CNN inference performance, and FSM states from the latest robot run.

Usage:
  python3 scripts/view_telemetry.py             # View latest run summary & recent driving rows
  python3 scripts/view_telemetry.py --tail 30   # Show last 30 driving frames
  python3 scripts/view_telemetry.py --events    # Show all detected driving events
  python3 scripts/view_telemetry.py --plot      # Plot steering angle, speed & path graphs (if matplotlib available)
  python3 scripts/view_telemetry.py /path/to/telemetry_file.csv
"""

import os
import sys
import csv
import glob
import argparse
from datetime import datetime


def find_workspace_dir():
    ws_dir = os.environ.get('WS_DIR', '')
    if ws_dir and os.path.isdir(ws_dir):
        return ws_dir
    curr = os.path.abspath(__file__)
    for _ in range(5):
        parent = os.path.dirname(curr)
        if os.path.isdir(os.path.join(parent, 'src')) or os.path.isdir(os.path.join(parent, 'logs')):
            return parent
        curr = parent
    for candidate in [
        '/home/vinh/Màn hình nền/robot_ws',
        os.path.expanduser('~/robot_ws'),
        os.path.expanduser('~/robot-ws')
    ]:
        if os.path.isdir(candidate):
            return candidate
    return os.path.expanduser('~')


def get_latest_files(logs_dir):
    csv_files = glob.glob(os.path.join(logs_dir, "telemetry_*.csv"))
    log_files = glob.glob(os.path.join(logs_dir, "system_run_*.log"))
    term_files = glob.glob(os.path.join(logs_dir, "terminal_real_cnn_*.log"))

    latest_csv = max(csv_files, key=os.path.getmtime) if csv_files else None
    latest_log = max(log_files, key=os.path.getmtime) if log_files else None
    latest_term = max(term_files, key=os.path.getmtime) if term_files else None

    # Check symlinks if present
    symlink_csv = os.path.join(logs_dir, "latest_telemetry.csv")
    if os.path.exists(symlink_csv):
        latest_csv = symlink_csv

    symlink_log = os.path.join(logs_dir, "latest_system_run.log")
    if os.path.exists(symlink_log):
        latest_log = symlink_log

    symlink_term = os.path.join(logs_dir, "terminal_real_cnn_latest.log")
    if os.path.exists(symlink_term):
        latest_term = symlink_term

    return latest_csv, latest_log, latest_term


def parse_csv(csv_path):
    if not os.path.isfile(csv_path):
        return [], []
    rows = []
    headers = []
    with open(csv_path, mode='r', encoding='utf-8', errors='replace') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []
        for row in reader:
            rows.append(row)
    return headers, rows


def analyze_run(rows):
    if not rows:
        return None

    steer_angles = []
    raw_steers = []
    confidences = []
    inference_times = []
    linear_speeds = []
    fsm_counts = {}
    total_dist = 0.0

    for r in rows:
        try:
            steer = float(r.get('Steer_Angle_deg', 0.0))
            steer_angles.append(steer)
        except (ValueError, TypeError):
            pass

        try:
            raw = float(r.get('Raw_Steer_deg', 0.0))
            raw_steers.append(raw)
        except (ValueError, TypeError):
            pass

        try:
            conf = float(r.get('Confidence', 0.0))
            confidences.append(conf)
        except (ValueError, TypeError):
            pass

        try:
            inf = float(r.get('Inference_ms', 0.0))
            if inf > 0:
                inference_times.append(inf)
        except (ValueError, TypeError):
            pass

        try:
            v = float(r.get('Linear_Vel_mps', 0.0))
            linear_speeds.append(v)
        except (ValueError, TypeError):
            pass

        try:
            d = float(r.get('Dist_Traveled_m', 0.0))
            if d > total_dist:
                total_dist = d
        except (ValueError, TypeError):
            pass

        st = r.get('FSM_State', 'UNKNOWN')
        fsm_counts[st] = fsm_counts.get(st, 0) + 1

    first_time = rows[0].get('Time', '')
    last_time = rows[-1].get('Time', '')

    duration_sec = 0.0
    try:
        t0 = float(rows[0].get('Unix_Timestamp', 0.0))
        t1 = float(rows[-1].get('Unix_Timestamp', 0.0))
        duration_sec = max(0.0, t1 - t0)
    except Exception:
        pass

    return {
        'total_frames': len(rows),
        'duration_sec': duration_sec,
        'start_time': first_time,
        'end_time': last_time,
        'distance_traveled': total_dist,
        'steer_min': min(steer_angles) if steer_angles else 0.0,
        'steer_max': max(steer_angles) if steer_angles else 0.0,
        'steer_mean': (sum(steer_angles) / len(steer_angles)) if steer_angles else 0.0,
        'steer_abs_mean': (sum(abs(s) for s in steer_angles) / len(steer_angles)) if steer_angles else 0.0,
        'conf_mean': (sum(confidences) / len(confidences)) if confidences else 0.0,
        'inf_mean': (sum(inference_times) / len(inference_times)) if inference_times else 0.0,
        'fps_mean': (1000.0 / (sum(inference_times) / len(inference_times))) if inference_times else 0.0,
        'speed_mean': (sum(linear_speeds) / len(linear_speeds)) if linear_speeds else 0.0,
        'fsm_counts': fsm_counts
    }


def print_summary(csv_path, log_path, term_path, stats, rows, tail_n=15):
    print("=" * 85)
    print(" 🌾 BÁO CÁO TRUY XUẤT DỮ LIỆU TỰ HÀNH AI XE THẬT (REAL-CNN TELEMETRY SUMMARY)")
    print("=" * 85)
    print(f"📁 Thư mục log        : {os.path.dirname(csv_path)}")
    print(f"📄 File dữ liệu CSV   : {csv_path}")
    if log_path and os.path.exists(log_path):
        print(f"📄 File sự kiện (.log): {log_path}")
    if term_path and os.path.exists(term_path):
        print(f"🖥️ File terminal run  : {term_path}")
    print("-" * 85)

    if not stats or stats.get('total_frames', 0) == 0:
        print("⚠️ [CHƯA CÓ FRAME DỮ LIỆU]: Không tìm thấy frame AI nào được ghi nhận trong phiên chạy này.")
        print("💡 Hướng dẫn kiểm tra:")
        print("   • Camera chưa gửi ảnh tới AI (chưa cắm cáp camera hoặc cổng /dev/video* bị kẹt).")
        print("   • Hoặc bạn vừa bật và tắt ngay lập tức (Ctrl+C) trước khi Camera kịp khởi động.")
        print("   • Bạn có thể kiểm tra hình ảnh Camera bằng lệnh: test-cam")
        print("=" * 85)
        return

    dur_min = int(stats['duration_sec'] // 60)
    dur_sec = stats['duration_sec'] % 60

    print("📊 [1] THÔNG SỐ VẬN HÀNH TỔNG QUAN:")
    print(f"   ⏱️ Thời gian bắt đầu   : {stats['start_time']} ➔ Kết thúc: {stats['end_time']}")
    print(f"   ⏳ Tổng thời lượng     : {dur_min} phút {dur_sec:.1f} giây ({stats['duration_sec']:.1f}s)")
    print(f"   📸 Tổng frame AI       : {stats['total_frames']} frames")
    print(f"   📏 Quãng đường di chuyển: {stats['distance_traveled']:.2f} mét")
    print(f"   ⚡ Tốc độ tiến TB       : {stats['speed_mean']:.2f} m/s")

    print("\n🎯 [2] PHÂN TÍCH GÓC LÁI VÀ AI CNN:")
    print(f"   🧭 Góc bẻ lái trung bình: {stats['steer_mean']:+.2f}° (Độ lệch tuyệt đối TB: {stats['steer_abs_mean']:.2f}°)")
    print(f"   ↔️ Biên độ góc lái      : Từ {stats['steer_min']:+.2f}° đến {stats['steer_max']:+.2f}°")
    print(f"   🧠 Độ tin cậy AI trung bình: {stats['conf_mean']*100:.1f}%")
    if stats['inf_mean'] > 0:
        print(f"   ⚡ Độ trễ suy luận AI TB  : {stats['inf_mean']:.1f} ms (~{stats['fps_mean']:.1f} FPS)")

    print("\n🔄 [3] PHÂN BỔ TRẠNG THÁI MÁY FSM (FINITE STATE MACHINE):")
    for state, count in stats['fsm_counts'].items():
        pct = (count / stats['total_frames']) * 100
        bar = "█" * int(pct // 4)
        print(f"   • [{state:^16s}]: {count:5d} frames ({pct:5.1f}%) | {bar}")

    # Display recent frames
    tail_rows = rows[-tail_n:] if len(rows) > tail_n else rows
    print(f"\n📋 [4] CHI TIẾT {len(tail_rows)} FRAME GẦN NHẤT:")
    header_fmt = "  {:<11} {:<16} {:>9} {:>8} {:>8} {:>7} {:>9} {:>10}"
    row_fmt    = "  {:<11} {:<16} {:>+9.2f}° {:>+8.2f}° {:>7.2f}m {:>6.1f}% {:>8.1f}ms {:>10}"
    print(header_fmt.format("Thời gian", "Trạng thái", "Góc lái", "Góc thô", "Vận tốc", "Tin cậy", "Độ trễ", "GPS Status"))
    print("  " + "-" * 81)
    for r in tail_rows:
        t = r.get('Time', '')
        st = r.get('FSM_State', 'UNKNOWN')
        try:
            steer = float(r.get('Steer_Angle_deg', 0.0))
        except Exception:
            steer = 0.0
        try:
            raw = float(r.get('Raw_Steer_deg', 0.0))
        except Exception:
            raw = 0.0
        try:
            v = float(r.get('Linear_Vel_mps', 0.0))
        except Exception:
            v = 0.0
        try:
            conf = float(r.get('Confidence', 0.0)) * 100
        except Exception:
            conf = 0.0
        try:
            inf = float(r.get('Inference_ms', 0.0))
        except Exception:
            inf = 0.0
        gps_st = r.get('GPS_Status', 'NO_FIX')
        print(row_fmt.format(t, st, steer, raw, v, conf, inf, gps_st))

    print("=" * 85)
    print("💡 MẸO TRUY XUẤT:")
    if log_path and os.path.exists(log_path):
        print(f"   • Xem chi tiết log sự kiện   : cat \"{log_path}\"")
    if term_path and os.path.exists(term_path):
        print(f"   • Xem toàn bộ terminal run  : cat \"{term_path}\"")
    print(f"   • Mở file CSV bằng LibreOffice / Excel: libreoffice \"{csv_path}\"")
    print("=" * 85)


def print_events(log_path):
    if not log_path or not os.path.exists(log_path):
        print(f"⚠️ Không tìm thấy file log sự kiện: {log_path}")
        return
    print("=" * 85)
    print(f"📜 NHẬT KÝ SỰ KIỆN LÁI XE VÀ ĐIỀU HƯỚNG TỪ: {os.path.basename(log_path)}")
    print("=" * 85)
    with open(log_path, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            if line.startswith("[") and "]" in line:
                print("  " + line.strip())
    print("=" * 85)


def try_plot(rows, output_png=None):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("⚠️ matplotlib chưa được cài đặt, bỏ qua chức năng vẽ đồ thị. (Cài: pip install matplotlib)")
        return

    if not rows or len(rows) == 0:
        print("⚠️ [CHƯA CÓ FRAME DỮ LIỆU]: Không có dữ liệu để vẽ đồ thị.")
        return

    times = []
    steers = []
    raw_steers = []
    offsets = []
    speeds = []
    confs = []
    xs = []
    ys = []

    t0 = None
    for r in rows:
        try:
            t = float(r.get('Unix_Timestamp', 0.0))
            if t0 is None:
                t0 = t
            rel_t = t - t0
        except Exception:
            continue

        try:
            steer = float(r.get('Steer_Angle_deg', 0.0))
            raw = float(r.get('Raw_Steer_deg', 0.0))
            offset = float(r.get('Lane_Offset_m', 0.0))
            v = float(r.get('Linear_Vel_mps', 0.0))
            c = float(r.get('Confidence', 0.0)) * 100.0
            x = float(r.get('Pos_X_m', 0.0))
            y = float(r.get('Pos_Y_m', 0.0))
        except Exception:
            continue

        times.append(rel_t)
        steers.append(steer)
        raw_steers.append(raw)
        offsets.append(offset)
        speeds.append(v)
        confs.append(c)
        xs.append(x)
        ys.append(y)

    if len(times) == 0:
        print("⚠️ [CHƯA CÓ FRAME DỮ LIỆU]: Dữ liệu frame rỗng hoặc không hợp lệ, không thể xuất đồ thị.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle('BÁO CÁO XU HƯỚNG TỰ HÀNH BÁM LUỐNG AI CNN (REAL-CNN TREND DASHBOARD)', fontsize=13, fontweight='bold')

    # 1. Steering Angle Trend
    ax1 = axes[0, 0]
    ax1.plot(times, steers, label='Góc lái mượt (°)', color='#1f77b4', linewidth=1.8)
    ax1.plot(times, raw_steers, label='Góc thô CNN (°)', color='#ff7f0e', alpha=0.45, linestyle='--')
    ax1.axhline(0.3, color='gray', linestyle=':', label='Ngưỡng căn IMU (±0.3°)')
    ax1.axhline(-0.3, color='gray', linestyle=':')
    ax1.set_xlabel('Thời gian (giây)')
    ax1.set_ylabel('Góc bẻ lái (°)')
    ax1.set_title('[1] Xu hướng Góc Lái theo Thời Gian')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=8)

    # 2. Lane Offset Trend
    ax2 = axes[0, 1]
    ax2.plot(times, offsets, label='Độ lệch tâm (m)', color='#d62728', linewidth=1.8)
    ax2.axhline(0.0, color='black', linestyle='-', linewidth=0.8)
    ax2.fill_between(times, -0.1, 0.1, color='green', alpha=0.12, label='Vùng an toàn tâm luống (±10cm)')
    ax2.set_xlabel('Thời gian (giây)')
    ax2.set_ylabel('Độ lệch (m)')
    ax2.set_title('[2] Xu hướng Độ Lệch Tâm Luống (Lane Offset)')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=8)

    # 3. Speed & Confidence Trend
    ax3 = axes[1, 0]
    ax3_conf = ax3.twinx()
    p1 = ax3.plot(times, speeds, label='Vận tốc (m/s)', color='#2ca02c', linewidth=1.6)
    p2 = ax3_conf.plot(times, confs, label='Độ tin cậy AI (%)', color='#9467bd', linestyle='-.', linewidth=1.5)
    ax3.set_xlabel('Thời gian (giây)')
    ax3.set_ylabel('Vận tốc (m/s)', color='#2ca02c')
    ax3_conf.set_ylabel('Confidence (%)', color='#9467bd')
    ax3_conf.set_ylim(0, 105)
    ax3.set_title('[3] Tốc Độ Di Chuyển & Độ Tin Cậy Mạng CNN')
    ax3.grid(True, linestyle=':', alpha=0.6)
    lines = p1 + p2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc='lower right', fontsize=8)

    # 4. 2D Trajectory X - Y
    ax4 = axes[1, 1]
    ax4.plot(xs, ys, label='Quỹ đạo xe (X, Y)', color='#e377c2', linewidth=2.0)
    if xs and ys:
        ax4.scatter([xs[0]], [ys[0]], color='green', s=60, label='Điểm xuất phát (Start)', zorder=5)
        ax4.scatter([xs[-1]], [ys[-1]], color='red', s=60, label='Điểm hiện tại (End)', zorder=5)
    ax4.set_xlabel('Tọa độ X (m - Hướng tiến)')
    ax4.set_ylabel('Tọa độ Y (m - Hướng ngang)')
    ax4.set_title('[4] Quỹ Đạo Di Chuyển 2D Thực Tế')
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(loc='best', fontsize=8)

    plt.tight_layout()
    ws_dir = find_workspace_dir()
    if output_png is None:
        output_png = os.path.join(ws_dir, 'logs', f"driving_plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
    latest_plot_png = os.path.join(ws_dir, 'logs', "latest_driving_plot.png")
    
    os.makedirs(os.path.dirname(output_png), exist_ok=True)
    plt.savefig(output_png, dpi=160)
    try:
        if os.path.islink(latest_plot_png) or os.path.exists(latest_plot_png):
            os.remove(latest_plot_png)
        os.symlink(os.path.basename(output_png), latest_plot_png)
    except Exception:
        try:
            import shutil
            shutil.copyfile(output_png, latest_plot_png)
        except Exception:
            pass
    print(f"📊 Đã xuất đồ thị xu hướng 4 bảng thành công: {output_png}")
    print(f"🖼️ Đồ thị mới nhất liên kết tại             : {latest_plot_png}")
    
    # Auto-open on desktop if DISPLAY is available
    if os.environ.get('DISPLAY') and not os.environ.get('SSH_CLIENT'):
        try:
            import subprocess
            subprocess.Popen(['xdg-open', latest_plot_png], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Xem và phân tích dữ liệu tự hành AI Real-CNN.")
    parser.add_argument('file', nargs='?', help="Đường dẫn file CSV hoặc log cần xem (mặc định lấy file mới nhất).")
    parser.add_argument('--tail', '-t', type=int, default=15, help="Số frame gần nhất cần in ra bảng (mặc định: 15).")
    parser.add_argument('--events', '-e', action='store_true', help="Chỉ xem nhật ký sự kiện chuyển trạng thái và cảnh báo.")
    parser.add_argument('--plot', '-p', action='store_true', help="Vẽ và lưu đồ thị phân tích góc lái và tốc độ ra file PNG.")
    args = parser.parse_args()

    ws_dir = find_workspace_dir()
    logs_dir = os.path.join(ws_dir, 'logs')

    latest_csv, latest_log, latest_term = get_latest_files(logs_dir)

    target_csv = latest_csv
    target_log = latest_log

    if args.file:
        if args.file.endswith('.csv'):
            target_csv = args.file
        elif args.file.endswith('.log'):
            target_log = args.file

    if not target_csv or not os.path.exists(target_csv):
        print(f"⚠️ Chưa tìm thấy file CSV telemetry nào trong thư mục: {logs_dir}")
        if target_log and os.path.exists(target_log):
            print_events(target_log)
        sys.exit(0)

    if args.events:
        print_events(target_log)
        sys.exit(0)

    headers, rows = parse_csv(target_csv)
    stats = analyze_run(rows)

    print_summary(target_csv, target_log, latest_term, stats, rows, tail_n=args.tail)

    if args.plot:
        try_plot(rows)


if __name__ == '__main__':
    main()

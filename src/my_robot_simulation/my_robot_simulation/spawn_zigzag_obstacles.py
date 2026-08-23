#!/usr/bin/env python3
"""
Spawn/Respawn dynamic zigzag obstacle pillars in Gazebo simulation.
Usage:
    ros2 run my_robot_simulation spawn_zigzag_obstacles
or
    python3 src/my_robot_simulation/my_robot_simulation/spawn_zigzag_obstacles.py
"""

import os
import sys
import subprocess
import time

OBSTACLES = [
    {
        "name": "obstacle_zigzag_1",
        "x": 1.80,
        "y": 0.65,
        "z": 0.30,
        "radius": 0.05,
        "length": 0.60,
        "color_ambient": "0.95 0.35 0.05 1",
        "color_diffuse": "0.95 0.35 0.05 1",
        "description": "Hàng 1 (X=1.80m, Y=0.65m - Lệch Trái: Robot né phải)"
    },
    {
        "name": "obstacle_zigzag_2",
        "x": 3.30,
        "y": 0.35,
        "z": 0.30,
        "radius": 0.05,
        "length": 0.60,
        "color_ambient": "0.95 0.75 0.05 1",
        "color_diffuse": "0.95 0.75 0.05 1",
        "description": "Hàng 1 (X=3.30m, Y=0.35m - Lệch Phải: Robot né trái)"
    },
    {
        "name": "obstacle_zigzag_3",
        "x": 3.00,
        "y": -0.35,
        "z": 0.30,
        "radius": 0.05,
        "length": 0.60,
        "color_ambient": "0.95 0.35 0.05 1",
        "color_diffuse": "0.95 0.35 0.05 1",
        "description": "Hàng 2 (X=3.00m, Y=-0.35m - Lệch Trái: Robot né phải)"
    },
    {
        "name": "obstacle_zigzag_4",
        "x": 1.50,
        "y": -0.65,
        "z": 0.30,
        "radius": 0.05,
        "length": 0.60,
        "color_ambient": "0.95 0.75 0.05 1",
        "color_diffuse": "0.95 0.75 0.05 1",
        "description": "Hàng 2 (X=1.50m, Y=-0.65m - Lệch Phải: Robot né trái)"
    }
]

def generate_sdf(obs):
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{obs['name']}">
    <static>true</static>
    <pose>{obs['x']} {obs['y']} {obs['z']} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{obs['radius']}</radius>
            <length>{obs['length']}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{obs['radius']}</radius>
            <length>{obs['length']}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{obs['color_ambient']}</ambient>
          <diffuse>{obs['color_diffuse']}</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

def main():
    print("=" * 70)
    print("🌾 SPAWNING ZIGZAG OBSTACLES IN GAZEBO CORN FIELD...")
    print("=" * 70)

    for i, obs in enumerate(OBSTACLES, 1):
        print(f"[{i}/{len(OBSTACLES)}] Spawning {obs['name']}: {obs['description']}...")
        sdf_str = generate_sdf(obs)
        
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "corn_field",
            "-name", obs["name"],
            "-string", sdf_str,
            "-x", str(obs["x"]),
            "-y", str(obs["y"]),
            "-z", str(obs["z"]),
            "-allow_renaming", "true"
        ]
        
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if res.returncode == 0:
                print(f"   ✅ Spawned successfully at ({obs['x']:.2f}m, {obs['y']:.2f}m)")
            else:
                print(f"   ⚠️ Result: {res.stdout.strip()} {res.stderr.strip()}")
        except Exception as e:
            print(f"   ❌ Error: {e}")
        time.sleep(0.2)

    print("\n✨ All zigzag obstacles processed!")
    print("=" * 70)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Generate stone_yard.world (+ stone_yard.json describing obstacles for tools/nav_test.py).

  python3 make_stone_yard.py [--seed 3]

Layout (robot spawns at 0,0 facing +x; suggested goal 14,0):
  gravel patch   x 1.5..4.5   40 small stones, 1.5-3.5 cm tall -> drivable (below the 6 cm step limit)
  large rocks    0.35-0.65 m tall, built from overlapping half-buried spheres -> must be avoided
  walker         a person walking back and forth across the path at x = 7.5 (0.6 m/s)
  cart           a box sliding back and forth across the path at x = 11 (0.4 m/s)
Moving objects are Gazebo actors: the camera sees them, but they have no collision -
tools/nav_test.py computes the clearance to them from their scripted trajectories.
"""
import argparse
import json
import math
import os
import random

MAT = ('<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
       '<name>{}</name></script></material>')

# name, x, y, list of (dx, dy, radius, height-above-ground)
ROCKS = [
    ('rock_1', 6.0, 0.2, [(0, 0, 0.55, 0.55), (0.35, 0.25, 0.35, 0.35), (-0.3, -0.2, 0.4, 0.40)]),
    ('rock_2', 6.4, -1.9, [(0, 0, 0.40, 0.40), (0.25, -0.2, 0.3, 0.30)]),
    ('rock_3', 9.0, 1.3, [(0, 0, 0.6, 0.65), (-0.4, -0.3, 0.35, 0.40), (0.3, -0.35, 0.3, 0.30)]),
    ('rock_4', 9.6, -1.0, [(0, 0, 0.35, 0.35), (0.25, 0.2, 0.25, 0.25)]),
    ('rock_5', 12.3, 0.3, [(0, 0, 0.45, 0.45), (-0.2, 0.35, 0.3, 0.30)]),
    ('rock_6', 4.2, 2.9, [(0, 0, 0.5, 0.50)]),
    ('rock_7', 3.6, -3.0, [(0, 0, 0.45, 0.45), (0.3, 0.1, 0.3, 0.3)]),
]

# moving objects: name, kind, waypoints [(t, x, y, yaw)], loop
WALKER = [(0, 7.5, -3.5, 1.5708), (11.7, 7.5, 3.5, 1.5708), (12.7, 7.5, 3.5, -1.5708),
          (24.4, 7.5, -3.5, -1.5708), (25.4, 7.5, -3.5, 1.5708)]
CART = [(0, 11.0, 3.0, 0.0), (15.0, 11.0, -3.0, 0.0), (17.0, 11.0, -3.0, 0.0),
        (32.0, 11.0, 3.0, 0.0), (34.0, 11.0, 3.0, 0.0)]


def sphere_rock(name, x, y, parts, mat='Gazebo/Grey'):
    """Each part is a sphere of radius r sunk so that it rises h above the ground."""
    vis = []
    for i, (dx, dy, r, h) in enumerate(parts):
        z = h - r
        geo = f'<geometry><sphere><radius>{r:.3f}</radius></sphere></geometry>'
        vis.append(f'<collision name="c{i}"><pose>{dx} {dy} {z:.3f} 0 0 0</pose>{geo}</collision>'
                   f'<visual name="v{i}"><pose>{dx} {dy} {z:.3f} 0 0 0</pose>{geo}{MAT.format(mat)}</visual>')
    return (f'<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 0</pose>'
            f'<link name="link">{"".join(vis)}</link></model>')


def waypoints(wps):
    return ''.join(f'<waypoint><time>{t}</time><pose>{x} {y} 0 0 0 {yaw}</pose></waypoint>'
                   for t, x, y, yaw in wps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=3)
    args = ap.parse_args()
    rng = random.Random(args.seed)
    here = os.path.dirname(os.path.abspath(__file__))

    # gravel: small stones on the direct route
    gravel = []
    stones = []
    while len(stones) < 40:
        x, y = rng.uniform(1.5, 4.5), rng.uniform(-1.3, 1.3)
        if any(math.hypot(x - sx, y - sy) < 0.18 for sx, sy, _, _ in stones):
            continue
        h = rng.uniform(0.015, 0.035)
        r = rng.uniform(0.04, 0.08)
        stones.append((x, y, r, h))
    for i, (x, y, r, h) in enumerate(stones):
        gravel.append(sphere_rock(f'pebble_{i:02d}', x, y, [(0, 0, max(r, h), h)], 'Gazebo/DarkGrey'))

    rocks = [sphere_rock(n, x, y, p) for n, x, y, p in ROCKS]

    walker = f'''
    <actor name="walker">
      <skin><filename>file://media/models/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walking"><filename>file://media/models/walk.dae</filename>
        <scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="walking">{waypoints(WALKER)}</trajectory>
      </script>
    </actor>'''

    cart = f'''
    <actor name="cart">
      <link name="link">
        <visual name="body"><pose>0 0 0.35 0 0 0</pose>
          <geometry><box><size>0.7 0.5 0.5</size></box></geometry>{MAT.format('Gazebo/Orange')}</visual>
        <visual name="handle"><pose>-0.4 0 0.75 0 0 0</pose>
          <geometry><box><size>0.05 0.5 0.05</size></box></geometry>{MAT.format('Gazebo/Black')}</visual>
      </link>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="slide">{waypoints(CART)}</trajectory>
      </script>
    </actor>'''

    def box(name, x, y, sx, sy, sz, mat):
        g = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        return (f'<model name="{name}"><static>true</static><pose>{x} {y} {sz / 2} 0 0 0</pose>'
                f'<link name="link"><collision name="c">{g}</collision>'
                f'<visual name="v">{g}{MAT.format(mat)}</visual></link></model>')

    walls = [box('wall_far', 17, 0, 0.3, 12, 2.0, 'Gazebo/Bricks'),
             box('fence_left', 8, 5.5, 18, 0.1, 1.0, 'Gazebo/WoodPallet'),
             box('fence_right', 8, -5.5, 18, 0.1, 1.0, 'Gazebo/WoodPallet')]

    world = f'''<?xml version="1.0"?>
<!-- GENERATED by make_stone_yard.py (seed {args.seed}) - edit the script, not this file. -->
<sdf version="1.6">
  <world name="stone_yard">
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>38.0</latitude_deg><longitude_deg>-78.5</longitude_deg>
      <elevation>150.0</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>
    <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.70 0.80 0.95 1</background><shadows>false</shadows></scene>
    <include><uri>model://sun</uri></include>
    <model name="ground"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="visual"><cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>{MAT.format('Gazebo/Grass')}</visual>
    </link></model>
    <!-- small stones (drivable) -->
    {chr(10).join('    ' + g for g in gravel)}
    <!-- large rocks (avoid) -->
    {chr(10).join('    ' + r for r in rocks)}
    <!-- boundary -->
    {chr(10).join('    ' + w for w in walls)}
    <!-- moving objects -->{walker}{cart}
  </world>
</sdf>
'''
    with open(os.path.join(here, 'stone_yard.world'), 'w') as f:
        f.write(world)

    # obstacle description for nav_test.py: circles for rocks, moving circles for actors
    info = {
        'static': [{'name': n, 'circles': [[x + dx, y + dy, math.sqrt(max(r * r - (r - h) ** 2, 0)) if h < r else r]
                                           for dx, dy, r, h in p]} for n, x, y, p in ROCKS],
        'boxes': [['wall_far', 17, 0, 0, 0.15, 6], ['fence_left', 8, 5.5, 0, 9, 0.05],
                  ['fence_right', 8, -5.5, 0, 9, 0.05]],
        'moving': [{'name': 'walker', 'radius': 0.3, 'waypoints': WALKER},
                   {'name': 'cart', 'radius': 0.43, 'waypoints': CART}],
        'pebbles': len(stones),
    }
    with open(os.path.join(here, 'stone_yard.json'), 'w') as f:
        json.dump(info, f, indent=1)
    print(f'wrote stone_yard.world ({len(stones)} pebbles, {len(ROCKS)} rocks, 2 movers)')


if __name__ == '__main__':
    main()

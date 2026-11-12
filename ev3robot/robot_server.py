#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EV3 room robot: pick a room on the phone map, joystick and speed control.

The map holds rooms, corridor points joined by lines, and yellow stickers.
Tapping a room plans the shortest way along the corridor lines from wherever
the robot is, and the robot drives it.

Runs on the EV3 under ev3dev (Python 3.5+, library ev3dev2).
On a normal computer (no ev3dev2) it runs a simulator, so the phone page
can be tried without the robot.

    On the EV3:       python3 robot_server.py
    On a computer:    python3 robot_server.py --sim

Then open on the phone:  http://<EV3 IP address>:8000

Hardware (same as the EV3 Classroom project):
    Motors B and D        tracks (physically B = right, D = left, mounted backwards)
    Port 1  ultrasonic    stop for obstacles closer than 15 cm
    Port 2  touch sensor  stop the route and turn 180 degrees
    Port 3  colour sensor yellow stickers correct the position on the map
    Port 4  gyro sensor   heading (which way the robot faces)

Map coordinates are centimetres from the top-left corner of the map,
x to the right, y down. Heading 0 = up on the map, clockwise positive.
"""

import json
import math
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(HERE, 'static')
SETTINGS_FILE = os.path.join(HERE, 'settings.json')
MAP_FILE = os.path.join(HERE, 'map.json')

LOOP_DT = 0.02          # control loop period, seconds (50 Hz)
JOY_TIMEOUT = 0.6       # stop if the phone stops sending joystick data
ARRIVE_CM = 3.0         # a waypoint counts as reached within this distance
YELLOW = 4              # ev3dev colour code for yellow

DEFAULT_SETTINGS = {
    # wiring
    'left_port': 'D',
    'right_port': 'B',
    'invert_motors': True,      # True: motor "forward" moves the robot backwards
    'gyro_sign': 1,             # change to -1 if the robot turns the wrong way on the map
    # geometry (calibrate on the Settings tab)
    'cm_per_rot': 12.0,         # how far the robot moves per motor rotation
    'track_width_cm': 12.0,     # only used when there is no gyro
    'sensor_offset_cm': 8.0,    # colour sensor distance in front of the turning centre
    # behaviour
    'speed': 40,                # % for routes and joystick
    'turn_speed': 25,           # % for turning on the spot
    'heading_kp': 1.5,          # how hard it steers back onto the line
    'obstacle_cm': 15,
    'clear_cm': 18,
    'snap_radius_cm': 40,       # a yellow sticker this close corrects the position
    'touch_action': 'turn180',  # 'turn180' or 'stop'
}
EDITABLE_SETTINGS = ('cm_per_rot', 'track_width_cm', 'sensor_offset_cm', 'speed',
                     'turn_speed', 'heading_kp', 'obstacle_cm', 'clear_cm',
                     'snap_radius_cm', 'touch_action', 'gyro_sign', 'invert_motors',
                     'left_port', 'right_port')
DEFAULT_MAP = {
    'image': '', 'width_cm': 300,
    'markers': [],      # yellow stickers [[x, y], ...]
    'rooms': [],        # [{'id': 'r1', 'name': '301', 'x': .., 'y': ..}, ...]
    'nodes': [],        # corridor points [{'id': 'n1', 'x': .., 'y': ..}, ...]
    'edges': [],        # corridor lines between points [['n1', 'n2'], ...]
}
MAP_KEYS = ('image', 'width_cm', 'markers', 'rooms', 'nodes', 'edges')


class RouteError(Exception):
    pass


def _project(p, a, b):
    """Closest point to p on the segment a-b, and the distance to it."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else clamp(((p[0] - ax) * dx + (p[1] - ay) * dy) / L2, 0.0, 1.0)
    q = (ax + t * dx, ay + t * dy)
    return q, math.hypot(p[0] - q[0], p[1] - q[1])


def plan_route(start, goal, nodes, edges):
    """Shortest way from start (x, y) to goal (x, y) along the corridor lines.

    The robot joins the corridor at the closest point of the nearest corridor
    line, follows the lines, and leaves at the line nearest to the goal.
    With no corridor points it drives straight to the goal.
    """
    if not nodes:
        return [list(goal)]
    pos = {n['id']: (float(n['x']), float(n['y'])) for n in nodes}
    ids = list(pos)
    segs = [(e[0], e[1]) for e in edges if e[0] in pos and e[1] in pos and e[0] != e[1]]
    inf = float('inf')
    dist = {a: {b: (0.0 if a == b else inf) for b in ids} for a in ids}
    nxt = {a: {b: (b if a == b else None) for b in ids} for a in ids}
    for a, b in segs:
        d = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
        if d < dist[a][b]:
            dist[a][b] = dist[b][a] = d
            nxt[a][b], nxt[b][a] = b, a
    for k in ids:                      # Floyd-Warshall: small maps only
        dk = dist[k]
        for i in ids:
            dik = dist[i][k]
            if dik == inf:
                continue
            di, ni = dist[i], nxt[i]
            for j in ids:
                v = dik + dk[j]
                if v < di[j]:
                    di[j] = v
                    ni[j] = ni[k]

    def attach(p):
        """Where p joins the corridor: (point on corridor, [candidate nodes], segment)."""
        if segs:
            best = None
            for a, b in segs:
                q, d = _project(p, pos[a], pos[b])
                if best is None or d < best[0]:
                    best = (d, q, a, b)
            return best[1], [best[2], best[3]], (best[2], best[3])
        n = min(ids, key=lambda i: math.hypot(p[0] - pos[i][0], p[1] - pos[i][1]))
        return pos[n], [n], None

    def L(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])

    ps, s_nodes, s_seg = attach(start)
    pg, g_nodes, g_seg = attach(goal)
    if s_seg is not None and g_seg is not None and set(s_seg) == set(g_seg):
        pts = [ps, pg, goal]          # same corridor line: no corridor points needed
    else:
        best = None
        for sn in s_nodes:
            for gn in g_nodes:
                if dist[sn][gn] == inf:
                    continue
                cost = L(ps, pos[sn]) + dist[sn][gn] + L(pos[gn], pg)
                if best is None or cost < best[0]:
                    best = (cost, sn, gn)
        if best is None:
            raise RouteError('corridor points are not connected')
        _, sn, gn = best
        chain = [sn]
        while chain[-1] != gn:
            chain.append(nxt[chain[-1]][gn])
        pts = [ps] + [pos[n] for n in chain] + [pg, goal]
    # drop points that are almost on top of the previous one (or of the robot)
    out = []
    prev = start
    for i, p in enumerate(pts):
        if L(p, prev) >= 5 or i == len(pts) - 1:
            out.append(p)
            prev = p
    return [[round(p[0], 1), round(p[1], 1)] for p in out]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap180(a):
    """Angle in degrees to the range (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def load_json(path, defaults):
    data = dict(defaults)
    try:
        with open(path, 'r') as f:
            data.update(json.load(f))
    except (IOError, OSError, ValueError):
        pass
    return data


def save_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f)
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Hardware: the real EV3, and a simulator with the same methods
# ---------------------------------------------------------------------------

class EV3Hardware(object):
    simulated = False

    def __init__(self, cfg):
        from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
        from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
        from ev3dev2.sensor.lego import UltrasonicSensor, TouchSensor, ColorSensor, GyroSensor
        from ev3dev2.sound import Sound
        self.SpeedPercent = SpeedPercent
        outs = {'A': OUTPUT_A, 'B': OUTPUT_B, 'C': OUTPUT_C, 'D': OUTPUT_D}
        self.left = LargeMotor(outs[cfg['left_port']])
        self.right = LargeMotor(outs[cfg['right_port']])
        self.sign = -1 if cfg['invert_motors'] else 1
        for m in (self.left, self.right):
            m.stop_action = 'brake'
        self.us = self._optional(UltrasonicSensor, INPUT_1, 'ultrasonic')
        self.touch = self._optional(TouchSensor, INPUT_2, 'touch')
        self.color = self._optional(ColorSensor, INPUT_3, 'colour')
        self.gyro = self._optional(GyroSensor, INPUT_4, 'gyro')
        if self.color:
            self.color.mode = 'COL-COLOR'
        if self.gyro:
            print('Gyro: keep still...')
            try:
                self.gyro.calibrate()
            except Exception:
                pass
            self.gyro.mode = 'GYRO-ANG'
            time.sleep(0.3)
        self.sound = Sound()
        self._last = [None, None]

    @staticmethod
    def _optional(cls, port, name):
        try:
            return cls(port)
        except Exception as e:
            print('No {} sensor: {}'.format(name, e))
            return None

    def drive(self, left_pct, right_pct):
        """Track speeds in % of maximum, + = robot forward."""
        for i, (motor, pct) in enumerate(((self.left, left_pct), (self.right, right_pct))):
            pct = clamp(pct * self.sign, -100, 100)
            if abs(pct) < 0.5:
                pct = 0
            last = self._last[i]
            if last is not None and abs(last - pct) < 0.5:
                continue
            self._last[i] = pct
            if pct == 0:
                motor.off(brake=True)
            else:
                motor.on(self.SpeedPercent(pct), block=False)

    def stop(self):
        self._last = [None, None]
        self.left.off(brake=True)
        self.right.off(brake=True)

    def wheel_degrees(self):
        return self.left.position * self.sign, self.right.position * self.sign

    def gyro_angle(self):
        return self.gyro.angle if self.gyro else None

    def distance_cm(self):
        try:
            return self.us.distance_centimeters if self.us else 255.0
        except Exception:
            return 255.0

    def touch_pressed(self):
        try:
            return bool(self.touch and self.touch.is_pressed)
        except Exception:
            return False

    def color_code(self):
        try:
            return self.color.color if self.color else 0
        except Exception:
            return 0

    def beep(self):
        try:
            self.sound.beep(play_type=1)
        except Exception:
            pass


class SimHardware(object):
    """Pretend robot for testing the phone page on a computer."""
    simulated = True
    MAX_DEG_S = 1000.0

    def __init__(self, cfg, robot_pose_ref):
        self.cfg = cfg
        self.true = robot_pose_ref       # dict with x, y, heading, markers (for the colour sensor)
        self.l_deg = 0.0
        self.r_deg = 0.0
        self.cmd = (0.0, 0.0)
        self.gyro = 0.0
        self.t = time.time()
        self.obstacle_until = 0

    def _advance(self):
        now = time.time()
        dt = min(now - self.t, 0.1)
        self.t = now
        l, r = self.cmd
        dl_deg = l / 100.0 * self.MAX_DEG_S * dt
        dr_deg = r / 100.0 * self.MAX_DEG_S * dt
        self.l_deg += dl_deg
        self.r_deg += dr_deg
        cm_rot = self.cfg['cm_per_rot']
        dl = dl_deg / 360.0 * cm_rot
        dr = dr_deg / 360.0 * cm_rot
        dth = math.degrees((dl - dr) / 12.0)
        h = math.radians(self.true['heading'] + dth / 2.0)
        d = (dl + dr) / 2.0
        self.true['x'] += d * math.sin(h)
        self.true['y'] -= d * math.cos(h)
        self.true['heading'] = wrap180(self.true['heading'] + dth)
        self.gyro += dth

    def drive(self, l, r):
        self._advance()
        self.cmd = (clamp(l, -100, 100), clamp(r, -100, 100))

    def stop(self):
        self.drive(0, 0)

    def wheel_degrees(self):
        self._advance()
        return self.l_deg, self.r_deg

    def gyro_angle(self):
        return int(round(self.gyro))

    def distance_cm(self):
        return 10.0 if time.time() < self.obstacle_until else 120.0

    def touch_pressed(self):
        return False

    def color_code(self):
        h = math.radians(self.true['heading'])
        off = self.cfg['sensor_offset_cm']
        sx = self.true['x'] + off * math.sin(h)
        sy = self.true['y'] - off * math.cos(h)
        for mx, my in self.true.get('markers', []):
            if abs(sx - mx) < 3 and abs(sy - my) < 3:
                return YELLOW
        return 6

    def beep(self):
        pass


# ---------------------------------------------------------------------------
# Robot brain: position tracking, routes, joystick
# ---------------------------------------------------------------------------

class Robot(object):
    def __init__(self, hw, cfg, mapdata):
        self.hw = hw
        self.cfg = cfg
        self.map = mapdata
        self.lock = threading.RLock()
        self.x, self.y, self.heading = 30.0, 30.0, 0.0
        self.mode = 'idle'           # idle | path | joystick | turn | distance
        self.path = []               # waypoints still to visit [[x, y], ...]
        self.seg_start = None
        self.phase = 'turn'          # path: 'turn' on the spot, then 'drive'
        self.turn_target = None
        self.dist_target = None
        self.dist_start = None
        self.joy = (0.0, 0.0)
        self.joy_time = 0.0
        self.paused = False
        self.clear_since = None
        self.obstacle = False
        self.us_cm = 255.0
        self.msg = 'Готов'
        self.trail = []
        self.target = ''
        self.running = True
        self._touch_was = False
        self._yellow_count = 0
        self._yellow_was = False
        self._last_wheels = hw.wheel_degrees()
        g = hw.gyro_angle()
        self._gyro_ref = g
        self._heading_ref = self.heading

    # ----- helpers -----
    def has_gyro(self):
        return self._gyro_ref is not None

    def set_pose(self, x, y, heading):
        with self.lock:
            self.x, self.y = float(x), float(y)
            self.heading = wrap180(float(heading))
            self._heading_ref = self.heading
            self._gyro_ref = self.hw.gyro_angle()
            self.trail = []
            self.msg = 'Положение робота задано'

    def stop(self, msg='Остановлен'):
        with self.lock:
            self.mode = 'idle'
            self.path = []
            self.paused = False
            self.hw.stop()
            self.msg = msg

    def go_to_room(self, room_id):
        with self.lock:
            room = None
            for r in self.map.get('rooms', []):
                if str(r.get('id')) == str(room_id) or str(r.get('name')) == str(room_id):
                    room = r
            if room is None:
                raise ValueError('нет такого кабинета')
            pts = plan_route((self.x, self.y), (float(room['x']), float(room['y'])),
                             self.map.get('nodes', []), self.map.get('edges', []))
            self.set_path(pts)
            self.target = room.get('name', '')
            self.msg = 'Еду в кабинет {}'.format(self.target)

    def set_path(self, points, append=False):
        pts = [[float(p[0]), float(p[1])] for p in points]
        with self.lock:
            if append and self.mode == 'path':
                self.path.extend(pts)
            else:
                self.path = pts
                self.target = ''
                self.mode = 'path'
                self.phase = 'turn'
                self.seg_start = [self.x, self.y]
                self.paused = False
            self.msg = 'Еду по маршруту'

    def set_joystick(self, jx, jy):
        with self.lock:
            if self.mode in ('path', 'turn', 'distance'):
                if abs(jx) < 0.05 and abs(jy) < 0.05:
                    return
                self.path = []
            self.joy = (clamp(float(jx), -1, 1), clamp(float(jy), -1, 1))
            self.joy_time = time.time()
            self.mode = 'joystick'
            self.msg = 'Джойстик'

    def turn_by(self, degrees):
        with self.lock:
            self.path = []
            self.turn_target = wrap180(self.heading + degrees)
            self.mode = 'turn'
            self.msg = 'Разворот'

    def drive_distance_test(self, rotations):
        with self.lock:
            self.path = []
            l, r = self.hw.wheel_degrees()
            self.dist_start = (l + r) / 2.0
            self.dist_target = rotations * 360.0
            self.turn_target = self.heading
            self.mode = 'distance'
            self.msg = 'Тест: {} оборота'.format(rotations)

    def state(self):
        with self.lock:
            return {
                'x': round(self.x, 1), 'y': round(self.y, 1),
                'heading': round(self.heading, 1),
                'mode': self.mode, 'paused': self.paused,
                'path': [[round(p[0], 1), round(p[1], 1)] for p in self.path],
                'speed': self.cfg['speed'],
                'obstacle': self.obstacle, 'us_cm': round(self.us_cm, 1),
                'msg': self.msg, 'gyro': self.has_gyro(),
                'sim': self.hw.simulated,
                'trail': self.trail[-200:],
                'target': self.target if self.mode == 'path' else '',
                't': time.time(),
            }

    # ----- control loop -----
    def loop(self):
        while self.running:
            t0 = time.time()
            try:
                with self.lock:
                    self._step()
            except Exception as e:
                self.hw.stop()
                self.msg = 'Ошибка: {}'.format(e)
                self.mode = 'idle'
                print('control error:', e)
            time.sleep(max(0.0, LOOP_DT - (time.time() - t0)))

    def _update_pose(self):
        l, r = self.hw.wheel_degrees()
        dl = (l - self._last_wheels[0]) / 360.0 * self.cfg['cm_per_rot']
        dr = (r - self._last_wheels[1]) / 360.0 * self.cfg['cm_per_rot']
        self._last_wheels = (l, r)
        old_h = self.heading
        g = self.hw.gyro_angle()
        if g is not None and self._gyro_ref is not None:
            self.heading = wrap180(self._heading_ref + self.cfg['gyro_sign'] * (g - self._gyro_ref))
        else:
            self.heading = wrap180(self.heading + math.degrees((dl - dr) / self.cfg['track_width_cm']))
        mid = math.radians(old_h + wrap180(self.heading - old_h) / 2.0)
        d = (dl + dr) / 2.0
        self.x += d * math.sin(mid)
        self.y -= d * math.cos(mid)
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 3:
            self.trail.append([round(self.x, 1), round(self.y, 1)])
            if len(self.trail) > 400:
                self.trail = self.trail[-300:]

    def _check_sensors(self):
        self.us_cm = self.hw.distance_cm()
        self.obstacle = self.us_cm < self.cfg['obstacle_cm']

        pressed = self.hw.touch_pressed()
        if pressed and not self._touch_was:
            if self.cfg.get('touch_action') == 'stop':
                self.stop('Кнопка: стоп')
            else:
                self.path = []
                self.turn_target = wrap180(self.heading + 180)
                self.mode = 'turn'
                self.msg = 'Кнопка: разворот 180°'
            self.hw.beep()
        self._touch_was = pressed

        yellow = self.hw.color_code() == YELLOW
        self._yellow_count = self._yellow_count + 1 if yellow else 0
        seen = self._yellow_count >= 3
        if seen and not self._yellow_was:
            self._snap_to_marker()
        self._yellow_was = seen if yellow else False

    def _snap_to_marker(self):
        h = math.radians(self.heading)
        off = self.cfg['sensor_offset_cm']
        sx = self.x + off * math.sin(h)
        sy = self.y - off * math.cos(h)
        best, best_d = None, self.cfg['snap_radius_cm']
        for m in self.map.get('markers', []):
            d = math.hypot(m[0] - sx, m[1] - sy)
            if d <= best_d:
                best, best_d = m, d
        if best is None:
            self.msg = 'Жёлтый стикер (нет на карте рядом)'
            return
        self.x = best[0] - off * math.sin(h)
        self.y = best[1] + off * math.cos(h)
        self.msg = 'Стикер: положение уточнено ({:.0f} см)'.format(best_d)
        self.hw.beep()

    def _turn_toward(self, target_heading):
        """Turn on the spot. Returns True when facing the target."""
        err = wrap180(target_heading - self.heading)
        if abs(err) < 2.5:
            self.hw.drive(0, 0)
            return True
        ts = self.cfg['turn_speed']
        p = clamp(abs(err) * 0.6, 8, ts) * (1 if err > 0 else -1)
        self.hw.drive(p, -p)
        return False

    def _obstacle_pause(self):
        """Pause while something is in front. Returns True while paused."""
        if self.obstacle:
            if not self.paused:
                self.msg = 'Препятствие — жду'
            self.paused = True
            self.clear_since = None
            self.hw.drive(0, 0)
            return True
        if self.paused:
            if self.us_cm < self.cfg['clear_cm']:
                self.hw.drive(0, 0)
                return True
            if self.clear_since is None:
                self.clear_since = time.time()
            if time.time() - self.clear_since < 0.5:
                self.hw.drive(0, 0)
                return True
            self.paused = False
            self.msg = 'Еду дальше'
        return False

    def _step(self):
        self._update_pose()
        self._check_sensors()

        if self.mode == 'joystick':
            if time.time() - self.joy_time > JOY_TIMEOUT:
                self.mode = 'idle'
                self.hw.drive(0, 0)
                self.msg = 'Готов'
                return
            jx, jy = self.joy
            if self.obstacle and jy > 0:
                jy = 0.0
            sp = self.cfg['speed']
            l, r = jy + jx, jy - jx
            m = max(1.0, abs(l), abs(r))
            self.hw.drive(l / m * sp, r / m * sp)

        elif self.mode == 'turn':
            if self._turn_toward(self.turn_target):
                self.mode = 'idle'
                self.msg = 'Развернулся'

        elif self.mode == 'distance':
            l, r = self.hw.wheel_degrees()
            done = (l + r) / 2.0 - self.dist_start
            if self._obstacle_pause():
                return
            if abs(done) >= self.dist_target:
                self.hw.drive(0, 0)
                self.mode = 'idle'
                self.msg = 'Тест окончен — измерьте расстояние'
                return
            err = wrap180(self.turn_target - self.heading)
            sp = self.cfg['speed']
            c = clamp(err * self.cfg['heading_kp'], -sp * 0.5, sp * 0.5)
            self.hw.drive(sp + c, sp - c)

        elif self.mode == 'path':
            self._follow_path()

        else:
            self.hw.drive(0, 0)

    def _follow_path(self):
        while self.path:
            tx, ty = self.path[0]
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            sx, sy = self.seg_start or [self.x, self.y]
            ux, uy = tx - sx, ty - sy
            seg = math.hypot(ux, uy)
            passed = seg > 1 and (dx * ux + dy * uy) < 0
            if dist < ARRIVE_CM or (passed and self.phase == 'drive'):
                self.seg_start = self.path.pop(0)
                self.phase = 'turn'
                continue
            break
        if not self.path:
            self.mode = 'idle'
            self.hw.drive(0, 0)
            self.msg = 'Приехал' + (' в кабинет {}'.format(self.target) if self.target else '')
            self.hw.beep()
            return

        # turning on the spot is allowed with something in front; driving is not
        if self.phase == 'drive' and self._obstacle_pause():
            return

        tx, ty = self.path[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dx, -dy))
        err = wrap180(bearing - self.heading)

        if self.phase == 'turn':
            if self._turn_toward(bearing):
                self.phase = 'drive'
            return
        if abs(err) > 45 and dist > 6:
            self.phase = 'turn'
            return
        sp = self.cfg['speed']
        # slow down near the last point
        if len(self.path) == 1 and dist < 15:
            sp = max(12, sp * dist / 15.0)
        c = clamp(err * self.cfg['heading_kp'], -sp * 0.6, sp * 0.6)
        self.hw.drive(sp + c, sp - c)

    def shutdown(self):
        self.running = False
        try:
            self.hw.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Web server for the phone
# ---------------------------------------------------------------------------

class ThreadedServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    robot = None
    sim_truth = None
    MAX_BODY = 6 * 1024 * 1024

    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype='application/json; charset=utf-8'):
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        if not isinstance(body, bytes):
            body = body.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n > self.MAX_BODY:
            raise ValueError('too large')
        raw = self.rfile.read(n) if n else b'{}'
        return json.loads(raw.decode('utf-8') or '{}')

    def do_GET(self):
        r = self.robot
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            try:
                with open(os.path.join(STATIC_DIR, 'index.html'), 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            except IOError:
                return self._send(404, {'error': 'static/index.html missing'})
        if path == '/api/state':
            return self._send(200, r.state())
        if path == '/api/map':
            return self._send(200, r.map)
        if path == '/api/settings':
            return self._send(200, r.cfg)
        return self._send(404, {'error': 'not found'})

    def do_POST(self):
        r = self.robot
        path = self.path.split('?')[0]
        try:
            data = self._body()
        except Exception as e:
            return self._send(400, {'error': str(e)})
        try:
            if path == '/api/joystick':
                r.set_joystick(data.get('x', 0), data.get('y', 0))
            elif path == '/api/goto':
                try:
                    r.go_to_room(data.get('room'))
                except RouteError:
                    return self._send(400, {'error': 'Точки коридора не соединены линиями'})
            elif path == '/api/path':
                pts = data.get('points') or []
                if not pts:
                    return self._send(400, {'error': 'empty route'})
                r.set_path(pts, bool(data.get('append')))
            elif path == '/api/stop':
                r.stop()
            elif path == '/api/speed':
                with r.lock:
                    r.cfg['speed'] = int(clamp(float(data.get('speed', 40)), 5, 100))
                save_json(SETTINGS_FILE, r.cfg)
            elif path == '/api/pose':
                r.set_pose(data['x'], data['y'], data.get('heading', 0))
                if self.sim_truth is not None:
                    self.sim_truth.update(x=float(data['x']), y=float(data['y']),
                                          heading=float(data.get('heading', 0)))
            elif path == '/api/turn':
                r.turn_by(float(data.get('degrees', 180)))
            elif path == '/api/testdrive':
                r.drive_distance_test(float(data.get('rotations', 3)))
            elif path == '/api/map':
                with r.lock:
                    for k in MAP_KEYS:
                        if k in data:
                            r.map[k] = data[k]
                    if self.sim_truth is not None:
                        self.sim_truth['markers'] = r.map.get('markers', [])
                save_json(MAP_FILE, r.map)
                return self._send(200, {'ok': True})
            elif path == '/api/settings':
                restart = False
                with r.lock:
                    for k in EDITABLE_SETTINGS:
                        if k in data:
                            v = data[k]
                            if k in ('left_port', 'right_port', 'invert_motors'):
                                restart = restart or r.cfg.get(k) != v
                            elif k not in ('touch_action',):
                                v = float(v)
                            r.cfg[k] = v
                save_json(SETTINGS_FILE, r.cfg)
                return self._send(200, {'ok': True, 'restart_needed': restart})
            elif path == '/api/sim_obstacle' and self.sim_truth is not None:
                r.hw.obstacle_until = time.time() + float(data.get('seconds', 3))
            else:
                return self._send(404, {'error': 'not found'})
        except (KeyError, TypeError, ValueError) as e:
            return self._send(400, {'error': str(e)})
        return self._send(200, r.state())


def local_ips():
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ips.append(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith('127.'):
                ips.append(ip)
    except Exception:
        pass
    return ips


def main():
    port = 8000
    if '--port' in sys.argv:
        port = int(sys.argv[sys.argv.index('--port') + 1])
    cfg = load_json(SETTINGS_FILE, DEFAULT_SETTINGS)
    mapdata = load_json(MAP_FILE, DEFAULT_MAP)

    truth = None
    hw = None
    if '--sim' not in sys.argv:
        try:
            hw = EV3Hardware(cfg)
        except ImportError:
            print('ev3dev2 not found: starting the SIMULATOR')
    if hw is None:
        truth = {'x': 30.0, 'y': 30.0, 'heading': 0.0, 'markers': mapdata.get('markers', [])}
        hw = SimHardware(cfg, truth)

    robot = Robot(hw, cfg, mapdata)
    Handler.robot = robot
    Handler.sim_truth = truth
    t = threading.Thread(target=robot.loop)
    t.daemon = True
    t.start()

    server = ThreadedServer(('', port), Handler)
    print('Robot ready.')
    for ip in local_ips() or ['<IP>']:
        print('Phone: http://{}:{}'.format(ip, port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()

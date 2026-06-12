#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from datetime import datetime
import sys

try:
    import rospy
    from upperlimb.msg import Joints
    from upperlimb.srv import MoveJByPath, MoveJByPathRequest,MoveJRequest,MoveJResponse
except Exception:
    rospy = None

NODE_NAME = "wave_wa1"
CALLS = [{'arm_type': 1,
  'payload': {'arm_type': 1,
              'is_async': False,
              'path': [{'joint': [-1.2667565761967126,
                                  1.0470857275686285,
                                  -0.13097559763536992,
                                  -1.7651733133452583,
                                  -1.4069600272659954,
                                  0.02685623343580917,
                                  0.038926721394753006]},
                       {'joint': [-1.2684223835049124,
                                  1.0464505636309696,
                                  0.5763813128623951,
                                  -1.6425832578728887,
                                  -1.203084387502713,
                                  0.026866022530682484,
                                  0.03892725534538246]},
                       {'joint': [-1.2667326077462349,
                                  1.0471096960191062,
                                  -0.13098758186060877,
                                  -1.7651731335818797,
                                  -1.4069480430407566,
                                  0.026856055452266017,
                                  0.038926721394753006]},
                       {'joint': [-1.2684463519553901,
                                  1.0464385794057307,
                                  0.576393297087634,
                                  -1.6425832578728887,
                                  -1.2031203401784296,
                                  0.026866022530682484,
                                  0.03892731467323018]},
                       {'joint': [-1.2667565761967126,
                                  1.047121680244345,
                                  -0.13097559763536992,
                                  -1.765173193503006,
                                  -1.4069839957164731,
                                  0.026856352091504603,
                                  0.03892654341120986]},
                       {'joint': [-1.2684463519553901,
                                  1.0463906425047753,
                                  0.5763213917362009,
                                  -1.642583018188384,
                                  -1.2030724032774742,
                                  0.026866022530682484,
                                  0.038927136689687024]},
                       {'joint': [-1.2667445919714737,
                                  1.0470617591181508,
                                  -0.13093964495965338,
                                  -1.765172953818501,
                                  -1.4069720114912343,
                                  0.026856352091504603,
                                  0.038926721394753006]},
                       {'joint': [-1.2684343677301513,
                                  1.0464745320814473,
                                  0.5763573444119174,
                                  -1.6425831380306364,
                                  -1.2030724032774742,
                                  0.02686596320283477,
                                  0.03892731467323018]}],
              'time': 16.0},
  'service': '/zj_humanoid/upperlimb/movej_by_path/left_arm'}]

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f'motion_seq_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')

def log_print(msg):
    now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    text = f'[{now_str}] {msg}'
    print(text)
    try:
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            f.write(text + '\n')
    except Exception:
        pass

def call_movej_by_path(service_name, payload):
    rospy.wait_for_service(service_name, timeout=10.0)
    client = rospy.ServiceProxy(service_name, MoveJByPath)
    req = MoveJByPathRequest()

    raw_path = payload.get('path', [])
    for item in raw_path:
        joints = item.get('joint', []) if isinstance(item, dict) else []
        jm = Joints()
        jm.joint = [float(v) for v in joints]
        req.path.append(jm)

    req.time = float(payload.get('time', 1.0))
    req.is_async = bool(payload.get('is_async', False))
    req.arm_type = int(payload.get('arm_type', 0))

    return client(req)

def main():
    if rospy is None:
        raise RuntimeError('请在ROS环境中运行该脚本')
    rospy.init_node(NODE_NAME, anonymous=True)
    log_print(f'节点启动: {NODE_NAME}')
    for idx, item in enumerate(CALLS, start=1):
        service = item.get('service')
        payload = item.get('payload')
        arm_type = item.get('arm_type')
        log_print(f'[INFO] {idx}/{len(CALLS)} 调用 {service}, arm_type={arm_type}')
        resp = call_movej_by_path(service, payload)
        ok = bool(getattr(resp, 'success', True))
        msg = str(getattr(resp, 'message', resp))
        if not ok:
            raise RuntimeError(f'服务返回失败: {service}, message={msg}')
        log_print(f'[OK] {service}: {msg}')

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'[ERR] {e}')
        sys.exit(1)

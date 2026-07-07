#!/usr/bin/env python3
"""
standalone/visualize.py
ROS2 없이 AGA 글러브 / Meta Quest 데이터를 Rerun으로 시각화.

사용법:
  python standalone/visualize.py [left|right] [spawn|connect] [atlas|meta_quest]

  spawn      : 로컬에서 Rerun Viewer 창을 직접 띄움 (기본값)
  connect    : 외부 Viewer에 연결 (Docker 등 원격 실행 시)
  atlas      : AGA 글러브 OSC 수신 (기본값)
  meta_quest : Meta Quest UDP 수신

의존성 (ROS2 제외):
  pip install numpy rerun-sdk python-osc pinocchio scipy
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from atlas_hand_core.config import OSC_CLIENT_PORT, OSC_LISTEN_IP, OSC_SERVER_PORT, OSC_TARGET_IP, QUEST_DISC_PORT, QUEST_UDP_PORT
from atlas_hand_core.hand_spherical_fk import HandRerunViz
from atlas_hand_core.sources import create_source

_MODELS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'models'))

def _parse_args():
    hand_type   = 'left'
    mode        = 'spawn'
    source_type = 'atlas'
    if len(sys.argv) > 1:
        hand_type = 'right' if sys.argv[1].lower() in ('right', 'r') else 'left'
    if len(sys.argv) > 2:
        mode = 'connect' if sys.argv[2].lower() == 'connect' else 'spawn'
    if len(sys.argv) > 3:
        source_type = sys.argv[3].lower()
    return hand_type, mode, source_type


def main():
    hand_type, mode, source_type = _parse_args()

    urdf_path = os.path.join(_MODELS_DIR, 'base_hand', 'urdf', f'{hand_type}.urdf')
    if not os.path.exists(urdf_path):
        print(f"[ERROR] URDF 없음: {urdf_path}")
        sys.exit(1)

    rr.init(f"AGA Hand ({hand_type.upper()})", spawn=(mode == 'spawn'))
    if mode == 'connect':
        rr.serve_grpc()

    rr.send_blueprint(rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="Hand Update View",
                origin="/world",
                contents=["/world/**", "/hand/**"],
            )
        )
    ))

    viz = HandRerunViz(hand_type=hand_type, urdf_path=urdf_path)
    viz.setup()

    latest_quats = [None]
    lock = threading.Lock()

    def _on_quat(quats: np.ndarray):
        with lock:
            latest_quats[0] = quats

    if source_type == 'atlas':
        source_kwargs = dict(
            listen_ip=OSC_LISTEN_IP,
            server_port=OSC_SERVER_PORT,
            target_ip=OSC_TARGET_IP,
            client_port=OSC_CLIENT_PORT,
        )
    elif source_type == 'meta_quest':
        source_kwargs = dict(
            listen_ip='0.0.0.0',
            udp_port=QUEST_UDP_PORT,
            disc_port=QUEST_DISC_PORT,
        )
    else:
        print(f"[ERROR] 알 수 없는 소스: '{source_type}'. 사용 가능: atlas, meta_quest")
        sys.exit(1)

    source = create_source(
        source_type,
        **source_kwargs,
        on_left_quat=_on_quat  if hand_type == 'left'  else None,
        on_right_quat=_on_quat if hand_type == 'right' else None,
    )
    source.start()
    print(f"소스: {source_type}  hand: {hand_type}")

    try:
        while True:
            with lock:
                quats = latest_quats[0].copy() if latest_quats[0] is not None else None
            if quats is not None:
                viz.update(quats)
            time.sleep(0.016)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        source.stop()


if __name__ == '__main__':
    main()

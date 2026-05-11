#!/usr/bin/env python3
"""
standalone/retarget.py
ROS2 없이 AGA 글러브 / Meta Quest → FK → IK 리타겟팅 파이프라인.

사용법:
  python standalone/retarget.py [left|right] [robot_config] [atlas|meta_quest]
  robot_config: base | robotis_hx5 | orca_hand  (기본: base)
  source:       atlas | meta_quest               (기본: atlas)

의존성 (ROS2 제외):
  pip install numpy scipy pinocchio dex-retargeting python-osc

결과 출력:
  - 콘솔에 조인트 각도(rad) 실시간 출력
  - 커스텀 처리가 필요하면 on_result() 함수를 수정하세요.
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from atlas_hand_core.config import OSC_CLIENT_PORT, OSC_LISTEN_IP, OSC_SERVER_PORT, OSC_TARGET_IP, QUEST_DISC_PORT, QUEST_UDP_PORT
from atlas_hand_core.hand_configs import CONFIG_REGISTRY
from atlas_hand_core.retargeter import HandRetargeter
from atlas_hand_core.sources import create_source

def _parse_args():
    hand_type   = 'left'
    config_name = 'base'
    source_type = 'atlas'
    if len(sys.argv) > 1:
        hand_type = 'right' if sys.argv[1].lower() in ('right', 'r') else 'left'
    if len(sys.argv) > 2:
        config_name = sys.argv[2]
    if len(sys.argv) > 3:
        source_type = sys.argv[3].lower()
    return hand_type, config_name, source_type


def on_result(joint_names: list, qpos: np.ndarray):
    """리타겟팅 결과 콜백. 필요에 따라 수정하세요."""
    pairs = {n: round(float(v), 4) for n, v in zip(joint_names, qpos)}
    print(f"\r{pairs}", end='', flush=True)


def main():
    hand_type, config_name, source_type = _parse_args()

    if config_name not in CONFIG_REGISTRY:
        print(f"[ERROR] 알 수 없는 robot_config '{config_name}'. 사용 가능: {list(CONFIG_REGISTRY.keys())}")
        sys.exit(1)

    retargeter = HandRetargeter(hand_type, config_name)
    print(f"리타겟팅 준비 완료 | hand={hand_type} | config={config_name} | source={source_type}")
    print(f"조인트: {retargeter.joint_names}\n")

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

    try:
        while True:
            with lock:
                quats = latest_quats[0].copy() if latest_quats[0] is not None else None
            if quats is not None:
                on_result(retargeter.joint_names, retargeter.compute(quats))
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        source.stop()


if __name__ == '__main__':
    main()

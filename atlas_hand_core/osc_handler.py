"""
atlas_hand_core/osc_handler.py
AGA 글러브 OSC 프로토콜 핸들러 (ROS2 비의존)

사용법:
    handler = AGAOscHandler(
        listen_ip='0.0.0.0', server_port=4040,
        target_ip='127.0.0.1', client_port=4042,
        on_left_quat=lambda q: ...,   # q: np.ndarray (17, 4)
        on_right_quat=lambda q: ...,
        on_status_change=lambda l, r: ...,  # l, r: bool
    )
    handler.start()
    ...
    handler.stop()
"""

import logging
import threading
import time
from typing import Callable, Optional

import numpy as np
from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

from atlas_hand_core.config import (
    AGA_FINGER_COUNT,
    AGA_RAW_FLOAT_COUNT,
    HEARTBEAT_INTERVAL_SEC,
    OSC_ADDR_LEFT_HAPT,
    OSC_ADDR_RIGHT_HAPT,
    OSC_MSG_TYPE_LEFT_HAPT,
    OSC_MSG_TYPE_RIGHT_HAPT,
    parse_aga_raw,
)

_log = logging.getLogger(__name__)


class AGAOscHandler:
    """AGA 글러브 OSC 프로토콜 핸들러.

    - OSC UDP 수신 서버 (pythonosc)
    - 하트비트 송신 루프
    - 햅틱 OSC 패킷 빌드 및 송신
    - 연결 상태 추적
    """

    def __init__(
        self,
        listen_ip: str,
        server_port: int,
        target_ip: str,
        client_port: int,
        on_left_quat:     Optional[Callable[[np.ndarray], None]] = None,
        on_right_quat:    Optional[Callable[[np.ndarray], None]] = None,
        on_status_change: Optional[Callable[[bool, bool], None]] = None,
        verbose: bool = False,
    ):
        self._listen_ip   = listen_ip
        self._server_port = server_port
        self._target_ip   = target_ip
        self._client_port = client_port

        self._on_left_quat     = on_left_quat
        self._on_right_quat    = on_right_quat
        self._on_status_change = on_status_change
        self._verbose = verbose

        self._left_connected  = False
        self._right_connected = False
        self._running  = False
        self._server   = None
        self._udp_client: Optional[SimpleUDPClient] = None

    # ─────────────────────────────────────────

    def start(self) -> None:
        """OSC 서버 + 하트비트 스레드 시작."""
        disp = osc_dispatcher.Dispatcher()
        disp.map('/left/quat/get',      self._h_left_quat)
        disp.map('/right/quat/get',     self._h_right_quat)
        disp.map('/device/status/get',  self._h_device_status)
        disp.map('/left/hapt/get',      self._h_echo)
        disp.map('/right/hapt/get',     self._h_echo)
        disp.map('/left/hapt/ret/get',  self._h_echo)
        disp.map('/right/hapt/ret/get', self._h_echo)
        disp.map('/device/alarm/get',   self._h_alarm)
        disp.map('/heartbeat/get',      self._h_echo)
        disp.set_default_handler(self._h_default)

        self._server = osc_server.ThreadingOSCUDPServer(
            (self._listen_ip, self._server_port), disp
        )
        self._running = True
        threading.Thread(
            target=self._server.serve_forever, daemon=True, name='OSCServer'
        ).start()

        self._udp_client = SimpleUDPClient(self._target_ip, self._client_port)
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='OSCHeartbeat'
        ).start()

        _log.info(f'OSC 수신 대기: {self._listen_ip}:{self._server_port}')
        _log.info(f'하트비트 -> {self._target_ip}:{self._client_port} (간격 {HEARTBEAT_INTERVAL_SEC}s)')

    def stop(self) -> None:
        """OSC 서버 종료."""
        self._running = False
        if self._server:
            self._server.shutdown()

    # ─────────────────────────────────────────

    def send_haptic(self, side: str, values: list) -> bool:
        """햅틱 OSC 송신.

        Returns:
            True — 송신 성공, False — 미연결로 생략
        """
        connected = self._left_connected if side == 'left' else self._right_connected
        if not connected:
            return False

        address  = OSC_ADDR_LEFT_HAPT   if side == 'left' else OSC_ADDR_RIGHT_HAPT
        msg_type = OSC_MSG_TYPE_LEFT_HAPT if side == 'left' else OSC_MSG_TYPE_RIGHT_HAPT

        packet: list = [msg_type]
        for i, v in enumerate(values[:AGA_FINGER_COUNT]):
            packet.append(i)
            packet.append(int(v))

        try:
            self._udp_client.send_message(address, packet)
            if self._verbose:
                _log.debug(f'[OSC] -> {side} hapt  {[int(v) for v in values[:AGA_FINGER_COUNT]]}')
            return True
        except Exception as e:
            _log.warning(f'[OSC] {side} 햅틱 전송 실패: {e}')
            return False

    @property
    def left_connected(self) -> bool:
        return self._left_connected

    @property
    def right_connected(self) -> bool:
        return self._right_connected

    # ─────────────────────────────────────────

    def _set_status(self, left_ok: bool, right_ok: bool) -> None:
        changed = (left_ok != self._left_connected or right_ok != self._right_connected)
        self._left_connected  = left_ok
        self._right_connected = right_ok
        if changed and self._on_status_change:
            self._on_status_change(left_ok, right_ok)

    def _h_left_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            _log.warning(f'[OSC] {address} 파싱 실패 (args={len(args)})')
            return
        if not self._left_connected:
            self._set_status(True, self._right_connected)
        if self._on_left_quat:
            self._on_left_quat(parse_aga_raw(raw))
        if self._verbose:
            _log.debug(f'[OSC] <- left quat  raw[:4]={raw[:4].round(3)}')

    def _h_right_quat(self, address: str, *args) -> None:
        raw = self._parse_floats(args, AGA_RAW_FLOAT_COUNT)
        if raw is None:
            _log.warning(f'[OSC] {address} 파싱 실패 (args={len(args)})')
            return
        if not self._right_connected:
            self._set_status(self._left_connected, True)
        if self._on_right_quat:
            self._on_right_quat(parse_aga_raw(raw))
        if self._verbose:
            _log.debug(f'[OSC] <- right quat  raw[:4]={raw[:4].round(3)}')

    def _h_device_status(self, address: str, *args) -> None:
        if len(args) < 3:
            return
        self._set_status(bool(args[1]), bool(args[2]))
        if self._verbose:
            _log.debug(f'[OSC] device status  left={self._left_connected}  right={self._right_connected}')

    def _h_echo(self, address: str, *args) -> None:
        if self._verbose:
            _log.debug(f'[OSC] <- {address}  args={args}')

    def _h_alarm(self, address: str, *args) -> None:
        _log.warning(f'[OSC] ALARM <- {address}  args={args}')

    def _h_default(self, address: str, *args) -> None:
        if self._verbose:
            _log.debug(f'[OSC] 미처리 주소: {address}  args={args}')

    def _heartbeat_loop(self) -> None:
        while self._running:
            try:
                self._udp_client.send_message('/device/status/get', '4')
                if self._verbose:
                    _log.debug('[OSC] -> heartbeat sent')
            except Exception as e:
                _log.warning(f'[OSC] heartbeat 전송 실패: {e}')
            time.sleep(HEARTBEAT_INTERVAL_SEC)

    @staticmethod
    def _parse_floats(args: tuple, count: int) -> Optional[np.ndarray]:
        if len(args) < count + 1:
            return None
        try:
            return np.array(args[1:count + 1], dtype=np.float32)
        except (TypeError, ValueError):
            return None

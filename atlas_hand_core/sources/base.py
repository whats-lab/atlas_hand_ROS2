"""
atlas_hand_core/sources/base.py
입력 소스 추상 인터페이스
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional

import numpy as np


class HandInputSource(ABC):
    """AGA 글러브 / Meta Quest 등 모든 입력 소스의 공통 인터페이스.

    구현체는 start() 이후 on_*_quat 콜백으로 17×4 쿼터니언 배열을 전달한다.
    """

    @abstractmethod
    def start(self) -> None:
        """수신 스레드 시작."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """수신 스레드 종료."""
        ...

    def send_haptic(self, side: str, values: list) -> bool:
        """햅틱 명령 송신. 지원하지 않는 소스는 False 반환."""
        return False

    @property
    def left_connected(self) -> bool:
        return False

    @property
    def right_connected(self) -> bool:
        return False

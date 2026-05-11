from typing import Dict, Type

from ._base import FingerChain, HandConfig
from .base_hand import BaseHandConfig
from .orca_hand import OrcaHandConfig
from .robotis_hx5_d20 import RobotisHX5Config
from .allegro_hand import AllegroHandConfig
from .leap_hand import LeapHandConfig
from .schunk_svh import SchunkSVHConfig
from .tesollo_dg5f import TesolloDG5FConfig
from .ability_hand import AbilityHandConfig

CONFIG_REGISTRY: Dict[str, Type[HandConfig]] = {
    "base_hand":       BaseHandConfig,
    "orca_hand":       OrcaHandConfig,
    "robotis_hx5_d20": RobotisHX5Config,
    "allegro_hand":    AllegroHandConfig,
    "leap_hand":       LeapHandConfig,
    "schunk_svh":      SchunkSVHConfig,
    "tesollo_dg5f":    TesolloDG5FConfig,
    "ability_hand":    AbilityHandConfig,
}

__all__ = [
    "HandConfig",
    "FingerChain",
    "CONFIG_REGISTRY",
    "BaseHandConfig",
    "OrcaHandConfig",
    "RobotisHX5Config",
    "AllegroHandConfig",
    "LeapHandConfig",
    "SchunkSVHConfig",
    "TesolloDG5FConfig",
    "AbilityHandConfig",
]

"""
atlas_hand_core/sources/__init__.py
입력 소스 팩토리
"""

from atlas_hand_core.sources.base import HandInputSource


def create_source(source_type: str, **kwargs) -> HandInputSource:
    """입력 소스 인스턴스 생성.

    Args:
        source_type: 'atlas' | 'meta_quest'
        **kwargs: 소스별 생성자 인자

    Atlas (AtlasGloveSource):
        listen_ip, server_port, target_ip, client_port,
        on_left_quat, on_right_quat, on_status_change, verbose

    Meta Quest (MetaQuestSource, 신 APK / OSC):
        listen_ip, osc_port, disc_port,
        on_left_quat, on_right_quat, on_status_change, verbose
    """
    if source_type == 'atlas':
        from atlas_hand_core.sources.atlas_glove import AtlasGloveSource
        return AtlasGloveSource(**kwargs)
    if source_type == 'meta_quest':
        from atlas_hand_core.sources.meta_quest import MetaQuestSource
        return MetaQuestSource(**kwargs)
    raise ValueError(f"알 수 없는 input_source: '{source_type}'. 사용 가능: 'atlas', 'meta_quest'")

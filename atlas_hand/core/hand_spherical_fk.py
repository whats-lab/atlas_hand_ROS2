"""
HandSphericalFK: Pinocchio spherical joint 기반 Hand FK

- 16개 spherical joint (손목 센서 0 제외)
- hand_data.json에서 kinematic 구조 로드

입력 쿼터니언: [x, y, z, w] (scipy 표준)
"""

import json
import os

import numpy as np
import pinocchio as pin
    
from scipy.spatial.transform import Rotation as ScipyR


# ─────────────────────────────────────────────
# 센서 1~16
# ─────────────────────────────────────────────
JOINT_ORDER = [
    'thumb_cmc0',  # sensor  1
    'thumb_cmc1',  # sensor  2 
    'thumb_mcp',   # sensor  3
    'thumb_ip',    # sensor  4
    'index_mcp',   # sensor  5
    'index_pip',   # sensor  6
    'index_dip',   # sensor  7
    'middle_mcp',  # sensor  8
    'middle_pip',  # sensor  9
    'middle_dip',  # sensor 10
    'ring_mcp',    # sensor 11
    'ring_pip',    # sensor 12
    'ring_dip',    # sensor 13
    'pinky_mcp',   # sensor 14
    'pinky_pip',   # sensor 15
    'pinky_dip',   # sensor 16
]

# URDF link suffix → Pinocchio joint suffix
URDF_TO_JOINT = {
    'wrist':        None,        
    'thumb_cmc0':   'thumb_cmc0',
    'thumb_cmc1':   'thumb_cmc1',
    'thumb_mcp':    'thumb_mcp',
    'thumb_ip':     'thumb_ip',
    'index_mcp':    'index_mcp',
    'index_pip':    'index_pip',
    'index_dip':    'index_dip',
    'middle_mcp':   'middle_mcp',
    'middle_pip':   'middle_pip',
    'middle_dip':   'middle_dip',
    'ring_mcp':     'ring_mcp',
    'ring_pip':     'ring_pip',
    'ring_dip':     'ring_dip',
    'pinky_mcp':    'pinky_mcp',
    'pinky_pip':    'pinky_pip',
    'pinky_dip':    'pinky_dip',
}


def _correct_quat(raw_xyzw: np.ndarray) -> np.ndarray:
    """센서 좌표계 보정"""
    return np.array([raw_xyzw[0], -raw_xyzw[1], -raw_xyzw[2], raw_xyzw[3]])

def build_model(json_path: str, hand_type: str = 'left') -> 'pin.Model':
    """
    hand_data.json으로부터 16개 spherical joint Pinocchio 모델 빌드.

    구조:
        universe (wrist root)
        ├── {pfx}thumb_cmc0   (spherical, sensor 1)
        │   └── {pfx}thumb_cmc1   (sensor 2)
        │       └── {pfx}thumb_mcp (sensor 3)
        │           └── {pfx}thumb_ip (sensor 4)
        ├── {pfx}index_mcp   (spherical, sensor 5)
        │   └── {pfx}index_pip → {pfx}index_dip
        ├── {pfx}middle_mcp  (sensor 8) → {pfx}middle_pip → {pfx}middle_dip
        ├── {pfx}ring_mcp    (sensor 11) → {pfx}ring_pip → {pfx}ring_dip
        └── {pfx}pinky_mcp   (sensor 14) → {pfx}pinky_pip → {pfx}pinky_dip
    """

    with open(json_path, 'r') as f:
        raw = json.load(f)

    joints = raw[hand_type]['joints']
    pfx = hand_type + '_'
    model = pin.Model()
    UNIVERSE = 0

    def add_sph(parent_id: int, suffix: str, xyz) -> int:
        placement = pin.SE3(np.eye(3), np.array(xyz, dtype=float))
        jid = model.addJoint(
            parent_id,
            pin.JointModelSpherical(),
            placement,
            pfx + suffix,
        )
        model.appendBodyToJoint(
            jid,
            pin.Inertia(1e-3, np.zeros(3), np.eye(3) * 1e-6),
            pin.SE3.Identity(),
        )
        return jid

    # ── Thumb (4 joints + 1 tip) ──────────────────────────────────────────
    cmc0_id  = add_sph(UNIVERSE, 'thumb_cmc0',  joints[f'{pfx}thumb_cmc0']['xyz'])
    cmc1_id  = add_sph(cmc0_id,  'thumb_cmc1',  joints[f'{pfx}thumb_cmc1_x']['xyz'])
    tmcp_id  = add_sph(cmc1_id,  'thumb_mcp',   joints[f'{pfx}thumb_mcp']['xyz'])
    tip_id   = add_sph(tmcp_id,  'thumb_ip',    joints[f'{pfx}thumb_ip']['xyz'])
    _        = add_sph(tip_id,   'thumb_tip',   [0.01486, 0.0, 0.0]) # Virtual tip

    # ── Index (3 joints + 1 tip) ──────────────────────────────────────────
    imcp_id = add_sph(UNIVERSE, 'index_mcp', joints[f'{pfx}index_mcp_z']['xyz'])
    ipip_id = add_sph(imcp_id,  'index_pip', joints[f'{pfx}index_pip']['xyz'])
    idip_id = add_sph(ipip_id,  'index_dip', joints[f'{pfx}index_dip']['xyz'])
    _       = add_sph(idip_id,  'index_tip', [0.01158, 0.0, 0.0]) # Virtual tip

    # ── Middle (3 joints + 1 tip) ─────────────────────────────────────────
    mmcp_id = add_sph(UNIVERSE, 'middle_mcp', joints[f'{pfx}middle_mcp_z']['xyz'])
    mpip_id = add_sph(mmcp_id,  'middle_pip', joints[f'{pfx}middle_pip']['xyz'])
    mdip_id = add_sph(mpip_id,  'middle_dip', joints[f'{pfx}middle_dip']['xyz'])
    _       = add_sph(mdip_id,  'middle_tip', [0.01220, 0.0, 0.0]) # Virtual tip

    # ── Ring (3 joints + 1 tip) ───────────────────────────────────────────
    rmcp_id = add_sph(UNIVERSE, 'ring_mcp', joints[f'{pfx}ring_mcp_z']['xyz'])
    rpip_id = add_sph(rmcp_id,  'ring_pip', joints[f'{pfx}ring_pip']['xyz'])
    rdip_id = add_sph(rpip_id,  'ring_dip', joints[f'{pfx}ring_dip']['xyz'])
    _       = add_sph(rdip_id,  'ring_tip', [0.01194, 0.0, 0.0]) # Virtual tip

    # ── Pinky (3 joints + 1 tip) ──────────────
    cmc_xyz  = np.array(joints[f'{pfx}pinky_0']['xyz'])
    pmcp_xyz = np.array(joints[f'{pfx}pinky_mcp_z']['xyz'])
    pmcp_id  = add_sph(UNIVERSE, 'pinky_mcp', (cmc_xyz + pmcp_xyz).tolist())
    ppip_id  = add_sph(pmcp_id,  'pinky_pip', joints[f'{pfx}pinky_pip']['xyz'])
    pdip_id  = add_sph(ppip_id,  'pinky_dip', joints[f'{pfx}pinky_dip']['xyz'])
    _        = add_sph(pdip_id,  'pinky_tip', [0.01191, 0.0, 0.0]) # Virtual tip

    return model


class HandSphericalFK:
    """
    Pinocchio spherical joint 기반 Hand FK.

    Usage:
        fk = HandSphericalFK('left')
        q, positions, rotations = fk.compute(sensor_quats_17)

        # positions: {joint_name: np.ndarray (3,)}  세계 좌표
        # rotations: {joint_name: np.ndarray (3,3)} 세계 회전 행렬
    """

    def __init__(self, hand_type: str = 'left', json_path: str = None):

        self.hand_type = hand_type.lower()
        self.pfx = self.hand_type + '_'

        if json_path is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                json_path = os.path.join(
                    get_package_share_directory('atlas_hand'), 'config', 'hand_data.json'
                )
            except Exception:
                # fallback for non-ROS environments
                json_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'hand_data.json')

        self.model = build_model(json_path, self.hand_type)
        self.data  = self.model.createData()

        # joint name → joint id
        self.name_to_jid = {
            self.model.names[i]: i
            for i in range(1, self.model.njoints)
        }

        # sensor index (1~16) → joint id
        self.sensor_to_jid = {
            s_idx + 1: self.name_to_jid[self.pfx + suffix]
            for s_idx, suffix in enumerate(JOINT_ORDER)
        }

        # 23-index 위치 배열 매핑
        self._joint_map = {
            1: self.pfx + 'thumb_cmc0', 2: self.pfx + 'thumb_cmc1', 3: self.pfx + 'thumb_mcp',
            4: self.pfx + 'thumb_ip',   5: self.pfx + 'thumb_tip',
            6: self.pfx + 'index_mcp',  7: self.pfx + 'index_pip',  8: self.pfx + 'index_dip',  9: self.pfx + 'index_tip',
            10: self.pfx + 'middle_mcp',11: self.pfx + 'middle_pip', 12: self.pfx + 'middle_dip',13: self.pfx + 'middle_tip',
            14: self.pfx + 'ring_mcp',  15: self.pfx + 'ring_pip',   16: self.pfx + 'ring_dip',  17: self.pfx + 'ring_tip',
            18: self.pfx + 'pinky_mcp', 19: self.pfx + 'pinky_mcp',  20: self.pfx + 'pinky_pip', 21: self.pfx + 'pinky_dip',
            22: self.pfx + 'pinky_tip',
        }

        print(f"[HandSphericalFK] {hand_type} hand loaded")
        print(f"  njoints={self.model.njoints}, nq={self.model.nq}")
        print(f"  joints: {list(self.model.names[1:])}")

    # ─────────────────────────────────────────
    def sensor_to_q(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        """
        17개 센서 쿼터니언 → Pinocchio q 벡터.

        Args:
            sensor_quats_17: shape (17, 4), 각 행 [x, y, z, w]
                sensor[0] = 손목 (이 함수에서는 무시, root는 origin 고정)
                sensor[1~16] = 각 관절
        Returns:
            q: (nq,) — spherical joint당 4개 (쿼터니언)
        """
        q = pin.neutral(self.model)   
        sign = 1 if self.hand_type == 'right' else -1 
            
        for sensor_idx, jid in self.sensor_to_jid.items():
            raw       = sensor_quats_17[sensor_idx]
            corrected = _correct_quat(raw)
            if self.hand_type == 'right':
                corrected[1] = -corrected[1]
                corrected[0] = -corrected[0]
            corrected = corrected / (np.linalg.norm(corrected) + 1e-9) 


            if sensor_idx == 14:
                _PINKY_CMC_OFFSET = ScipyR.from_euler('y', sign * np.radians(20.0))
                corrected = (_PINKY_CMC_OFFSET * ScipyR.from_quat(corrected)).as_quat()

            idx_q = self.model.idx_qs[jid]
            q[idx_q: idx_q + 4] = corrected   

        return q

    # ─────────────────────────────────────────
    def compute(self, sensor_quats_17: np.ndarray):
        """
        FK 실행.

        Returns:
            q:         Pinocchio q 벡터
            positions: {joint_name: np.ndarray (3,)}  세계 좌표
            rotations: {joint_name: np.ndarray (3,3)} 세계 회전 행렬
        """
        q = self.sensor_to_q(sensor_quats_17)
        pin.forwardKinematics(self.model, self.data, q)

        positions = {}
        rotations = {}
        for jid in range(1, self.model.njoints):
            name = self.model.names[jid]
            oMi  = self.data.oMi[jid]
            positions[name] = oMi.translation.copy()
            rotations[name] = oMi.rotation.copy()

        return q, positions, rotations

    # ─────────────────────────────────────────
    def compute_positions(self, sensor_quats_17: np.ndarray) -> np.ndarray:
        """
        FK 실행 후 23-index 위치 배열 반환.

        Returns:
            positions: shape (23, 3), index 0=wrist(origin), 1~22=joints
        """
        _, pos_dict, _ = self.compute(sensor_quats_17)
        out = np.zeros((23, 3), dtype=np.float64)
        for i, name in self._joint_map.items():
            if name in pos_dict:
                out[i] = pos_dict[name]
        return out


class HandRerunViz:
    """
    URDF STL 메쉬를 Rerun으로 시각화.

    Usage:
        rr.init("hand_viz", spawn=True)

        viz = HandRerunViz(
            hand_type    = 'left',
            urdf_path    = '/path/to/left_hand_rerun.urdf',
            mesh_base_dir= '/path/to/left_hand',
        )
        viz.setup()               # 메쉬 한 번 등록 (static)
        viz.update(quats_17)      # 매 프레임 transform 갱신
    """

    def __init__(self, hand_type: str, urdf_path: str, mesh_base_dir: str,
                 json_path: str = None, entity_prefix: str = 'hand'):
        self.fk            = HandSphericalFK(hand_type, json_path)
        self.urdf_path     = urdf_path
        self.mesh_base_dir = mesh_base_dir.rstrip('/')
        self.entity_prefix = entity_prefix
        pfx                = hand_type + '_'

        # 전체 link name → joint name 매핑
        self.link_to_jname = {}
        for link_suf, joint_suf in URDF_TO_JOINT.items():
            if joint_suf is None:
                continue
            self.link_to_jname[pfx + link_suf] = pfx + joint_suf

    # ─────────────────────────────────────────
    def setup(self):
        """
        URDF STL 메쉬를 Rerun에 정적 등록.

        엔티티 구조:
            {prefix}/{link_name}         ← FK Transform3D (update()에서 갱신)
            {prefix}/{link_name}/visual  ← visual origin Transform3D + Asset3D (static)
        """
        try:
            import rerun as rr
            import xml.etree.ElementTree as ET
        except ImportError as e:
            raise ImportError(f"pip install rerun-sdk: {e}")

        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_X_UP, static=True)

        rr.log(
            "world/origin",
            rr.Arrows3D(
                vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                labels=["X", "Y", "Z"]
            ),
            static=True
        )

        rr.log(
            "world/camera",
            rr.Transform3D(
                translation=[0.2, -0.2, 0.2],
                rotation=rr.Quaternion(xyzw=[0.4, 0.2, 0.1, 0.9])
            ),
            static=True
        )

        pfx  = self.fk.pfx
        tree = ET.parse(self.urdf_path)
        n_logged = 0

        for visual in tree.getroot().iter('visual'):
            vname     = visual.get('name', '')
            link_name = vname.replace('_visual', '')

            is_wrist  = (link_name == pfx + 'wrist')
            is_mapped = (link_name in self.link_to_jname)
            if not is_wrist and not is_mapped:
                continue

            mesh_elem = visual.find('geometry/mesh')
            if mesh_elem is None:
                continue

            filename = mesh_elem.get('filename', '')
            filename = filename.replace('package://left_hand/',  self.mesh_base_dir + '/')
            filename = filename.replace('package://right_hand/', self.mesh_base_dir + '/')
            if not os.path.exists(filename):
                print(f"[WARN] 메쉬 없음: {filename}")
                continue

            scale = [float(s) for s in mesh_elem.get('scale', '1 1 1').split()]

            origin = visual.find('origin')
            if origin is not None:
                vis_xyz  = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                vis_rpy  = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                vis_quat = ScipyR.from_euler('xyz', vis_rpy).as_quat()
            else:
                vis_xyz  = [0., 0., 0.]
                vis_quat = [0., 0., 0., 1.]

            link_entity   = f'{self.entity_prefix}/{link_name}'
            visual_entity = f'{link_entity}/visual'

            rr.log(visual_entity, rr.Transform3D(
                translation=vis_xyz,
                rotation=rr.Quaternion(xyzw=vis_quat),
                scale=scale,
            ), static=True)

            rr.log(visual_entity, rr.Asset3D(path=filename), static=True)

            if is_wrist:
                rr.log(link_entity, rr.Transform3D(
                    translation=[0., 0., 0.],
                    rotation=rr.Quaternion(xyzw=[0., 0., 0., 1.]),
                ), static=True)

            axis_length = 0.01
            rr.log(
                f"{link_entity}/axes",
                rr.Arrows3D(
                    vectors=[[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
                static=True
            )

            n_logged += 1

        print(f"[HandRerunViz] 메쉬 등록 완료 ({n_logged}개 링크, 손목 포함)")

    # ─────────────────────────────────────────
    def update(self, sensor_quats_17: np.ndarray, timestamp: float = None):
        """
        FK 계산 후 Rerun Transform3D 갱신.

        Args:
            sensor_quats_17: shape (17, 4), 각 행 [x, y, z, w]
            timestamp:       선택적 시각 (초 단위)
        """
        try:
            import rerun as rr
        except ImportError:
            raise ImportError("pip install rerun-sdk")

        if timestamp is not None:
            rr.set_time('time', timestamp=timestamp)

        _, positions, rotations = self.fk.compute(sensor_quats_17)

        for link_name, joint_name in self.link_to_jname.items():
            if joint_name not in positions:
                continue

            pos  = positions[joint_name]
            quat = ScipyR.from_matrix(rotations[joint_name]).as_quat()

            rr.log(
                f'{self.entity_prefix}/{link_name}',
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=quat),
                ),
            )


"""G4 快照 ABI，与本项目 g4_shim.cpp 的 g4_joint_size 同步检查。"""
import ctypes


class G4Joint(ctypes.Structure):
    _fields_ = [("bus", ctypes.c_uint32), ("id", ctypes.c_uint32), ("valid", ctypes.c_uint32),
                ("error", ctypes.c_uint32), ("angle_deg", ctypes.c_float),
                ("current_a", ctypes.c_float), ("temp_c", ctypes.c_float),
                ("speed_rpm", ctypes.c_float), ("hz", ctypes.c_float),
                ("samples", ctypes.c_uint64), ("received_ns", ctypes.c_int64),
                ("board_timestamp_us", ctypes.c_uint32), ("max_board_gap_us", ctypes.c_uint32),
                ("max_host_gap_us", ctypes.c_uint32), ("tx_failures", ctypes.c_uint32)]

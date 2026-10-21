"""Unitree J288/S288 サーボの通信パケットと, 1 回送って 1 回受け取る通信クラス.

公式の python/servo_demo.py の代わりに使う. 関数名, 引数, 戻り値の辞書のキーは
servo_demo.py と同じにしてある. パケットの並びは, 公式のファームウェアの構造体
(external/digital_servo/stm32/firmware/App/protocol.h) に合わせた.

servo_demo.py との違い:
    - 目標位置 pos_des を符号付き (int32) で詰める. servo_demo.py は符号なしで詰めるので,
      負の目標位置で struct.error になる.
    - 巻線温度 sensor と電圧 vol を符号なし (uint8) で読む. servo_demo.py は符号付きで読むので,
      生値が 128 以上 (128 °C 以上, 64 V 以上) で負になる.
"""

import math
import struct

import serial

RATIO = 70070.0 / 243.0                         # 減速比

SERIAL_PORT = "/dev/tty.usbmodem36594BB319741"  # 公式の USB アダプタ
BAUD_RATE = 4000000                             # サーボは公式ツールで 4 Mbps に設定しておく

FEEDBACK_LEN = 26


def _crc_table():
    table = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = (c << 1) ^ 0x04C11DB7 if c & 0x80000000 else c << 1
        table.append(c & 0xFFFFFFFF)
    return table


CRC_TABLE = _crc_table()


def crc32(data):
    """CRC-32 (多項式 0x04C11DB7, 初期値 0xFFFFFFFF, ビット反転と最後の XOR なし).

    4 バイトずつを little-endian の 32 bit 語とみなし, 上位バイトから計算する
    (STM32 の CRC 回路に語を書き込むのと同じ). 4 バイトに満たない末尾は使わない.
    """
    crc = 0xFFFFFFFF
    for i in range(0, len(data) - 3, 4):
        for b in reversed(data[i:i + 4]):
            crc = CRC_TABLE[(crc >> 24) ^ b] ^ ((crc << 8) & 0xFFFFFFFF)
    return crc


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def build_control_packet(motor_id, mode, timeout, outputTor, outputSpd, outputPos, Kp, Kd):
    """制御パケット (20 バイト). 物理量の範囲と生値への換算は公式の specs/protocol.md のとおり."""
    mode_byte = _clamp(motor_id, 0, 15) | _clamp(mode, 0, 1) << 4 | (1 if timeout else 0) << 7
    tor = round(_clamp(outputTor, -36.9093, 36.908174) / RATIO * 256000.0)
    spd = round(_clamp(outputSpd, -278.90143, 278.90143) * RATIO * 2.560 / math.pi / 2.0)
    pos = round(_clamp(outputPos, -1428.018898, 1428.018898) * RATIO * 32768.0 / math.pi / 2.0)
    k_pos = round(_clamp(Kp, 0.0, 2128.523254) / (RATIO * RATIO) * 1280000.0)
    k_spd = round(_clamp(Kd, 0.0, 21.285233) / (RATIO * RATIO) * 128000000.0)
    body = b"\xFE\xEE" + bytes([mode_byte, 0]) + struct.pack(
        "<hhihh", _clamp(tor, -32768, 32767), _clamp(spd, -32768, 32767),
        _clamp(pos, -2 ** 31, 2 ** 31 - 1), _clamp(k_pos, 0, 32767), _clamp(k_spd, 0, 32767))
    return body + struct.pack("<I", crc32(body))


def parse_feedback_packet(data):
    """応答パケット (26 バイト) を物理量の辞書にする. 長さ, 先頭の 2 バイト, CRC のどれかが合わなければ None."""
    if len(data) != FEEDBACK_LEN or data[0] != 0xFC or data[1] != 0xEE:
        return None
    if struct.unpack("<I", data[22:26])[0] != crc32(data[2:22]):    # CRC は先頭の 2 バイトを含まない
        return None
    mode_byte = data[2]
    temp, sensor, vol, torque, speed, pos, merror, outpos, _, _ = struct.unpack("<bBBhhiIHBB", data[3:22])
    return {
        "motor_id": mode_byte & 0x0F,
        "mode": (mode_byte >> 4) & 0x07,
        "timeout": mode_byte >> 7,
        "Temp": temp,
        "sensor": sensor,
        "vol": vol / 2.0,
        "MError": merror,
        "MWarn": outpos >> 13,
        "ExPos": 2 * math.pi * (outpos & 0x1FFF) / 8192.0,
        "outputTor": (torque / 256000.0) * RATIO,
        "outputSpd": (speed / 2.56) * 2 * math.pi / RATIO,
        "outputPos": 2 * math.pi * pos / 32768.0 / RATIO,
    }


class MotorProtocolSync:
    """制御パケットを 1 個送り, 応答を 1 個読む."""

    def __init__(self, port, baudrate, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def send_and_receive(self, motor_id, mode, timeout_flag, outputTor, outputSpd, outputPos, Kp, Kd):
        """応答を辞書で返す. 応答が足りないか壊れていれば, 理由を表示して None を返す."""
        self.ser.write(build_control_packet(motor_id, mode, timeout_flag, outputTor, outputSpd, outputPos, Kp, Kd))
        reply = self.ser.read(FEEDBACK_LEN)
        if len(reply) != FEEDBACK_LEN:
            print(f"[エラー] 応答が {len(reply)} バイトしか届かなかった")
            return None
        parsed = parse_feedback_packet(reply)
        if parsed is None:
            print("[エラー] 応答の先頭か CRC が合わない")
        return parsed

    def close(self):
        if self.ser.is_open:
            self.ser.close()

"""バス上のサーボを 1 台ずつ出力端エンコーダのゼロ点 (ExPos = 0) へ戻し, 到達を確認するスクリプト.

ゼロ点の基準について:
    outputPos は転子側の単回転絶対エンコーダ + 電源投入後の回転数カウントから求めた値で,
    電源を切ると回転数がリセットされる (= 0 は電源投入時の位置で, 毎回変わる).
    一方 ExPos は出力端の絶対エンコーダ (13 bit, 2π/8192 rad) で, 電源に関係なく
    出力軸の機械的な角度を示す. 本スクリプトは ExPos = 0 をゼロ点とする.

ID 0-14 を順に処理する (ID 15 はブロードキャストで応答が返らず成否を確認できないため対象外).
各 ID について:

    1. 問い合わせ (mode=0) で応答の有無と現在の ExPos / outputPos を確認する. 応答がなければスキップ
    2. MError が非 0, またはゼロ点までの移動量 (最短経路, ±π 以内) が --max-dist を超えればスキップ
    3. outputPos 座標で「現在位置 + ExPos 誤差」を目標にし, 最小躍度軌道 (速度上限 --speed) で
       mode=1, timeout=1 で移動する. 移動中に ExPos と outputPos の回転方向が一致するか確認する
    4. 軌道終了後は ExPos の誤差を少しずつ目標に足し込み (ギアのガタ等を補正),
       |ExPos 誤差| < --tol が 0.1 s 続けば成功. --settle 秒以内に入らなければ失敗として mode=0 を送る
    5. 成功したサーボは timeout=0 で目標位置の保持指令を送り, そのまま保持させる
       (--release 指定時は mode=0 を送る)

最後に全 ID の結果を一覧表示する.
Ctrl+C で中断した場合, 動かしていたサーボには mode=0 を送る.

注意: mode=0 はマニュアル上「锁定」であり, 軸が自由に回る (脱力) かどうかは実機で要確認.

使用例:
    python return_to_zero.py
    python return_to_zero.py --ids 0 1 2 --speed 0.5 --kp 20
"""

import argparse
import math
import time

import serial

from servo_demo import (SERIAL_PORT, BAUD_RATE, MotorProtocolSync,
                        build_control_packet, parse_feedback_packet)

MODE_STOP = 0
MODE_CLOSED_LOOP = 1
BROADCAST_ID = 15
EXPOS_LSB = 2 * math.pi / 8192   # 出力端エンコーダの分解能 [rad]
DIR_CHECK_MOVE = 0.05            # 方向一致を確認する移動量 [rad]
SETTLE_SAMPLES_S = 0.1           # 許容誤差内に留まる必要がある時間 [s]
CORRECTION_GAIN = 0.02           # ExPos 誤差を毎周期目標に足し込む割合


def parse_args():
    p = argparse.ArgumentParser(description="全サーボを出力端エンコーダのゼロ点へ戻して到達を確認する")
    p.add_argument("--port", default=SERIAL_PORT, help="シリアルポート")
    p.add_argument("--ids", type=int, nargs="+", default=list(range(BROADCAST_ID)),
                   help="対象 ID (既定: 0-14)")
    p.add_argument("--kp", type=float, default=10.0, help="剛性ゲイン Kp (0-2128.523)")
    p.add_argument("--kd", type=float, default=0.3, help="減衰ゲイン Kd (0-21.285)")
    p.add_argument("--speed", type=float, default=1.0, help="移動時の最大速度 [rad/s]")
    p.add_argument("--tol", type=float, default=0.02,
                   help=f"到達判定の許容誤差 [rad] (ExPos 分解能 {EXPOS_LSB:.5f})")
    p.add_argument("--settle", type=float, default=2.0,
                   help="軌道終了後, 許容誤差に入るまで待つ時間 [s]")
    p.add_argument("--max-dist", type=float, default=math.pi,
                   help="ゼロ点までの移動量 (最短経路) がこれを超えるサーボは動かさない [rad]")
    p.add_argument("--rate", type=float, default=200.0, help="制御周期 [Hz]")
    p.add_argument("--release", action="store_true",
                   help="到達確認後に保持せず mode=0 を送る")
    return p.parse_args()


def wrap(a):
    """角度を [-π, π) に丸める."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class ServoBus:
    """応答なしを想定した 1 往復通信. 未接続 ID のたびにエラー表示が出ないよう,
    MotorProtocolSync.send_and_receive を使わず直接送受信する."""

    def __init__(self, port):
        self.motor = MotorProtocolSync(port, BAUD_RATE, timeout=0.02)

    def transact(self, motor_id, mode, timeout_flag, spd=0.0, pos=0.0, kp=0.0, kd=0.0):
        ser = self.motor.ser
        ser.reset_input_buffer()
        ser.write(build_control_packet(motor_id, mode, timeout_flag, 0.0, spd, pos, kp, kd))
        fb = parse_feedback_packet(ser.read(26))
        if fb is None or fb["motor_id"] != motor_id:
            return None
        return fb

    def stop(self, motor_id):
        return self.transact(motor_id, MODE_STOP, 0)

    def close(self):
        self.motor.close()


def min_jerk(s):
    """0→1 の最小躍度補間. (位置比, 速度比 d/ds) を返す."""
    s = min(max(s, 0.0), 1.0)
    return (10 * s**3 - 15 * s**4 + 6 * s**5,
            30 * s**2 - 60 * s**3 + 30 * s**4)


def return_one(bus, motor_id, args):
    """1 台をゼロ点に戻す. (状態文字列, 最終 ExPos 誤差 or None) を返す."""
    fb = bus.stop(motor_id)
    if fb is None:
        return "応答なし", None
    ex0 = fb["ExPos"]
    err0 = wrap(-ex0)
    if fb["MError"] != 0:
        return f"スキップ (MError={fb['MError']:#x})", err0
    if abs(err0) > args.max_dist:
        return f"スキップ (移動量 {abs(err0):.2f} rad > --max-dist)", err0

    # 指令は outputPos 座標で送るので, ExPos の誤差を outputPos の目標に換算する
    start = fb["outputPos"]
    goal = start + err0
    move_time = max(0.5, 1.875 * abs(err0) / args.speed)  # 最小躍度軌道のピーク速度 = 1.875 * 距離 / 時間
    print(f"  ID {motor_id}: ExPos {ex0:.4f} rad → 0 (移動 {err0:+.4f} rad) を {move_time:.2f} s で移動")

    period = 1.0 / args.rate
    need_in_tol = max(1, int(round(SETTLE_SAMPLES_S * args.rate)))
    in_tol = 0
    dir_checked = False
    err = err0
    t0 = time.perf_counter()
    next_tick = t0
    missed = 0
    while True:
        t = time.perf_counter() - t0
        if t > move_time + args.settle:
            bus.stop(motor_id)
            return f"失敗 (ExPos 誤差 {abs(err):.4f} rad > --tol)", err

        if t < move_time:
            r, dr = min_jerk(t / move_time)
            pos_des = start + (goal - start) * r
            spd_des = (goal - start) * dr / move_time
        else:
            pos_des, spd_des = goal, 0.0
        fb = bus.transact(motor_id, MODE_CLOSED_LOOP, 1, spd_des, pos_des, args.kp, args.kd)
        if fb is None:
            missed += 1
            if missed >= 10:
                bus.stop(motor_id)
                return "失敗 (移動中に応答途絶)", err
        else:
            missed = 0
            if fb["MError"] != 0:
                bus.stop(motor_id)
                return f"失敗 (MError={fb['MError']:#x})", err
            err = wrap(-fb["ExPos"])

            # 2 つのエンコーダの回転方向が逆だと補正が発散するので, 動き始めで確認する
            d_out = fb["outputPos"] - start
            if not dir_checked and abs(d_out) > DIR_CHECK_MOVE:
                d_ex = wrap(fb["ExPos"] - ex0)
                if d_out * d_ex <= 0.0:
                    bus.stop(motor_id)
                    return (f"失敗 (エンコーダ方向不一致: outputPos {d_out:+.3f}, "
                            f"ExPos {d_ex:+.3f} rad)"), err
                dir_checked = True

            if t >= move_time:
                if abs(err) < args.tol:
                    in_tol += 1
                    if in_tol >= need_in_tol:
                        break
                else:
                    in_tol = 0
                    # ギアのガタ等で残る ExPos 誤差を少しずつ目標に足し込む
                    goal += CORRECTION_GAIN * err

        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0.0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()

    if args.release:
        bus.stop(motor_id)
        return "成功 (mode=0)", err
    # timeout=0: 以後の送信がなくても目標位置の保持を続ける
    if bus.transact(motor_id, MODE_CLOSED_LOOP, 0, 0.0, goal, args.kp, args.kd) is None:
        bus.stop(motor_id)
        return "失敗 (保持指令に応答なし)", err
    return "成功 (保持中)", err


def main():
    args = parse_args()
    if any(not 0 <= i < BROADCAST_ID for i in args.ids):
        raise SystemExit("--ids は 0-14 で指定してください (15 はブロードキャストで応答なし)")
    if args.speed <= 0.0 or args.rate <= 0.0:
        raise SystemExit("--speed と --rate は正の値を指定してください")

    bus = None
    results = {}
    current = None
    try:
        bus = ServoBus(args.port)
        print(f"ID {args.ids} を順に出力端エンコーダのゼロ点へ戻します (Ctrl+C で中断)")
        for motor_id in args.ids:
            current = motor_id
            results[motor_id] = return_one(bus, motor_id, args)
            current = None
            print(f"  ID {motor_id}: {results[motor_id][0]}")
    except serial.SerialException as e:
        print(f"シリアルエラー: {e}")
    except KeyboardInterrupt:
        print("\nユーザ中断")
    finally:
        if bus:
            try:
                if current is not None:
                    bus.stop(current)
                    results[current] = ("中断 (mode=0 を送信)", None)
            finally:
                bus.close()

    print("\n==== 結果 (ExPos 誤差) ====")
    for motor_id in args.ids:
        status, err = results.get(motor_id, ("未実行", None))
        err_s = f"{err:+9.4f} rad" if err is not None else f"{'-':>13s}"
        print(f"ID {motor_id:2d}: {err_s}  {status}")
    ok = sum(s.startswith("成功") for s, _ in results.values())
    found = sum(s != "応答なし" for s, _ in results.values())
    print(f"応答あり {found} 台 / 成功 {ok} 台")
    if ok and not args.release:
        print("成功したサーボはゼロ点を保持し続けます. 解除するには mode=0 を送ってください")


if __name__ == "__main__":
    main()

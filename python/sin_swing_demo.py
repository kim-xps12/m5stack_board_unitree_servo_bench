"""位置制御モードで sin 波に沿ってサーボをスイングさせるサンプル.

servo_protocol.py の通信クラス (MotorProtocolSync) を使い, mode=1 (hybrid closed-loop)
で以下の目標値を一定周期で送り続ける.

    pos_des(t) = center + A * env(t) * sin(2π f t)
    spd_des(t) = A * env(t) * 2π f * cos(2π f t)   (速度フィードフォワード)

サーボ内部では概ね  tau = tor_des + Kp (pos_des - pos) + Kd (spd_des - spd)  が
実行される. 起動時の出力軸位置を中心 (center) とし, 振幅は ramp 秒かけて 0 から
立ち上げるので, 開始時に目標値が跳ぶことはない.

timeout=1 (超時保護有効) で送信するため, プログラムが止まればサーボも約 1 s で停止する.
終了時 (Ctrl+C 含む) は mode=0 を送ってサーボを停止する.

使用例:
    python sin_swing_demo.py --amp 0.5 --freq 0.5 --kp 10 --kd 0.3
"""

import argparse
import math
import time

import serial

from servo_protocol import SERIAL_PORT, BAUD_RATE, MotorProtocolSync

MODE_STOP = 0
MODE_CLOSED_LOOP = 1


def parse_args():
    p = argparse.ArgumentParser(description="位置制御 sin 波スイングデモ")
    p.add_argument("--port", default=SERIAL_PORT, help="シリアルポート")
    p.add_argument("--id", type=int, default=0, help="サーボ ID (0-14)")
    p.add_argument("--amp", type=float, default=0.5, help="振幅 [rad] (出力軸)")
    p.add_argument("--freq", type=float, default=0.5, help="周波数 [Hz]")
    p.add_argument("--kp", type=float, default=10.0, help="剛性ゲイン Kp (0-2128.523)")
    p.add_argument("--kd", type=float, default=0.3, help="減衰ゲイン Kd (0-21.285)")
    p.add_argument("--duration", type=float, default=10.0,
                   help="スイング時間 [s] (0 以下で Ctrl+C まで継続)")
    p.add_argument("--ramp", type=float, default=1.0, help="振幅の立ち上げ/立ち下げ時間 [s]")
    p.add_argument("--rate", type=float, default=200.0, help="制御周期 [Hz]")
    p.add_argument("--print-rate", type=float, default=10.0, help="状態表示の頻度 [Hz]")
    return p.parse_args()


def envelope(t, duration, ramp):
    """振幅エンベロープ (0→1→0). 立ち上げ・立ち下げは cos 補間で滑らかにする."""
    if ramp <= 0.0:
        return 1.0
    k = min(1.0, t / ramp)
    if duration > 0.0:
        k = min(k, max(0.0, (duration - t) / ramp))
    return 0.5 - 0.5 * math.cos(math.pi * k)


def stop(motor, motor_id):
    motor.send_and_receive(motor_id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)


def main():
    args = parse_args()
    if args.freq <= 0.0 or args.rate <= 0.0:
        raise SystemExit("--freq と --rate は正の値を指定してください")

    motor = None
    try:
        motor = MotorProtocolSync(args.port, BAUD_RATE, timeout=0.2)

        # 現在位置を取得してスイング中心にする (mode=0: 停止状態のまま読み出し)
        fb = motor.send_and_receive(args.id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if fb is None:
            raise SystemExit("サーボから応答がありません. 配線・ID・ポートを確認してください")
        center = fb["outputPos"]
        print(f"中心位置 = {center:.4f} rad, 電圧 = {fb['vol']:.1f} V, 温度 = {fb['Temp']} ℃")
        print(f"スイング開始: A={args.amp} rad, f={args.freq} Hz, Kp={args.kp}, Kd={args.kd}"
              f"  (Ctrl+C で停止)")

        omega = 2.0 * math.pi * args.freq
        period = 1.0 / args.rate
        print_every = max(1, int(round(args.rate / args.print_rate)))
        # duration 指定時は立ち下げ分まで含めてその時間で終える
        end_time = args.duration if args.duration > 0.0 else math.inf

        t0 = time.perf_counter()
        next_tick = t0
        n = 0
        while True:
            t = time.perf_counter() - t0
            if t >= end_time:
                break

            a = args.amp * envelope(t, args.duration, args.ramp)
            pos_des = center + a * math.sin(omega * t)
            spd_des = a * omega * math.cos(omega * t)

            fb = motor.send_and_receive(args.id, MODE_CLOSED_LOOP, 1,
                                        0.0, spd_des, pos_des, args.kp, args.kd)
            if fb is not None and n % print_every == 0:
                err = pos_des - fb["outputPos"]
                print(f"t={t:6.2f}s  des={pos_des:+.4f}  pos={fb['outputPos']:+.4f}  "
                      f"err={err:+.4f} rad  tor={fb['outputTor']:+.3f} Nm  "
                      f"MError={fb['MError']}")
            n += 1

            # 固定周期で送信 (遅れた場合は取り戻さずに次周期へ)
            next_tick += period
            sleep = next_tick - time.perf_counter()
            if sleep > 0.0:
                time.sleep(sleep)
            else:
                next_tick = time.perf_counter()

        elapsed = time.perf_counter() - t0
        print(f"完了: {n} 周期 / {elapsed:.2f} s (平均 {n / elapsed:.1f} Hz)")
    except serial.SerialException as e:
        print(f"シリアルエラー: {e}")
    except KeyboardInterrupt:
        print("\nユーザ中断")
    finally:
        if motor:
            try:
                stop(motor, args.id)
                print("サーボ停止 (mode=0)")
            finally:
                motor.close()


if __name__ == "__main__":
    main()

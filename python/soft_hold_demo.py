"""低 Kp の位置制御で「その場で止まり, 外力で柔らかくバックドライブできる」状態を試すサンプル.

servo_protocol.py の通信クラス (MotorProtocolSync) を使い, mode=1 (hybrid closed-loop)
で起動時の出力軸位置を目標に保持する.

    tau = Kp (pos_hold - pos) + Kd (0 - spd)

Kp を下げるほど, 手で押したときに柔らかいバネのように動き, 離すと元の位置へ戻る.
Kp=0 にすると保持力はなくなり, Kd による粘性 (ダンパ) だけが残る.

実行中は保持位置からの変位, 出力トルク, 見かけのバネ定数 (トルク / 変位) を表示する.
timeout=1 (超時保護有効) で送信し, 終了時 (Ctrl+C 含む) は mode=0 で停止する.

使用例:
    python soft_hold_demo.py --kp 1.0 --kd 0.05
    python soft_hold_demo.py --kp 0 --kd 0.2      # ダンパのみ
"""

import argparse
import time

import serial

from servo_protocol import SERIAL_PORT, BAUD_RATE, RATIO, MotorProtocolSync

MODE_STOP = 0
MODE_CLOSED_LOOP = 1

# build_control_packet の固定小数点変換における 1 LSB あたりのゲイン
KP_LSB = RATIO * RATIO / 1280000.0     # ≈ 0.065 [Nm/rad]
KD_LSB = RATIO * RATIO / 128000000.0   # ≈ 0.00065 [Nm·s/rad]


def parse_args():
    p = argparse.ArgumentParser(description="低 Kp 位置保持 (バックドライブ) デモ")
    p.add_argument("--port", default=SERIAL_PORT, help="シリアルポート")
    p.add_argument("--id", type=int, default=0, help="サーボ ID (0-14)")
    p.add_argument("--kp", type=float, default=1.0, help="剛性ゲイン Kp (0-2128.523)")
    p.add_argument("--kd", type=float, default=0.05, help="減衰ゲイン Kd (0-21.285)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="保持時間 [s] (0 以下で Ctrl+C まで継続)")
    p.add_argument("--rate", type=float, default=200.0, help="制御周期 [Hz]")
    p.add_argument("--print-rate", type=float, default=10.0, help="状態表示の頻度 [Hz]")
    return p.parse_args()


def quantized(value, lsb):
    return round(value / lsb) * lsb


def stop(motor, motor_id):
    motor.send_and_receive(motor_id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)


def main():
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate は正の値を指定してください")

    # ゲインは整数に丸めて送られるので, 実際に効く値を事前に示す
    kp_eff = quantized(args.kp, KP_LSB)
    kd_eff = quantized(args.kd, KD_LSB)
    print(f"実効ゲイン: Kp={kp_eff:.4f} (分解能 {KP_LSB:.4f}), "
          f"Kd={kd_eff:.5f} (分解能 {KD_LSB:.5f})")
    if args.kp > 0.0 and kp_eff == 0.0:
        print("[警告] Kp が分解能未満のため 0 に丸められます (保持力なし)")

    motor = None
    try:
        motor = MotorProtocolSync(args.port, BAUD_RATE, timeout=0.2)

        # 現在位置を保持目標にする (mode=0: 停止状態のまま読み出し)
        fb = motor.send_and_receive(args.id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if fb is None:
            raise SystemExit("サーボから応答がありません. 配線・ID・ポートを確認してください")
        hold = fb["outputPos"]
        print(f"保持位置 = {hold:.4f} rad, 電圧 = {fb['vol']:.1f} V, 温度 = {fb['Temp']} ℃")
        print("出力軸を手で押してみてください (Ctrl+C で停止)")

        period = 1.0 / args.rate
        print_every = max(1, int(round(args.rate / args.print_rate)))
        max_defl = 0.0

        t0 = time.perf_counter()
        next_tick = t0
        n = 0
        while args.duration <= 0.0 or time.perf_counter() - t0 < args.duration:
            fb = motor.send_and_receive(args.id, MODE_CLOSED_LOOP, 1,
                                        0.0, 0.0, hold, args.kp, args.kd)
            if fb is not None:
                defl = fb["outputPos"] - hold
                max_defl = max(max_defl, abs(defl))
                if n % print_every == 0:
                    tor = fb["outputTor"]
                    # 変位が小さいときは比が不安定なので表示しない
                    k_app = f"{-tor / defl:7.3f}" if abs(defl) > 0.02 else "    ---"
                    print(f"t={time.perf_counter() - t0:6.2f}s  defl={defl:+.4f} rad  "
                          f"spd={fb['outputSpd']:+.3f} rad/s  tor={tor:+.3f} Nm  "
                          f"k_app={k_app} Nm/rad  MError={fb['MError']}")
            n += 1

            # 固定周期で送信 (遅れた場合は取り戻さずに次周期へ)
            next_tick += period
            sleep = next_tick - time.perf_counter()
            if sleep > 0.0:
                time.sleep(sleep)
            else:
                next_tick = time.perf_counter()

        print(f"完了: 最大変位 {max_defl:.4f} rad")
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

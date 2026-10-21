"""出力トルクを実測するためのスクリプト (出力軸を固定した状態で使う).

mode=1 で Kp=0, spd_des=0 とし, トルク指令 tor_des だけでサーボを駆動する.
--kd を与えると速度に比例したブレーキ (Kd * (0 - spd)) が加わる. 出力軸が止まっていれば
トルクに影響せず, 動いた場合は自由回転の速度がおおよそ |目標トルク| / Kd で頭打ちになる.
トルク指令は 0 → 目標値へ --ramp 秒で上げて保持し, --ramp 秒で 0 へ戻す.
保持は --hold 秒, 省略時は Ctrl+C を押すまで押し続ける (Ctrl+C で立ち下げ, 2 回目で即停止).
その間のフィードバック (outputTor, 速度, 位置, 温度, 電圧, MError) を記録し,
最後に「指令値に対してどこまでトルクが出たか」を集計する.

    --torque を指定した場合: その値 (符号付き) を目標にする
    --torque を省略した場合: プロトコル上の最大トルク (±36.9 Nm) を目標にする (向きは --dir)

安全策:
    - 出力軸が固定されていない (速度が --max-speed 超 / 移動量が --max-move 超) と判断したら即停止
      (--allow-motion でこの判定を無効化. その場合は --kd で速度を抑えることを推奨)
    - 巻線温度が --max-temp を超えたら停止. MError が非 0 になったら停止
    - timeout=1 で送信し, 終了時は mode=0 で停止
    - 開始前に Enter での確認を求める (--yes で省略)

注意: outputTor はサーボ内部の推定値 (電流からの換算) で, トルクセンサによる実測ではない.
外部のトルクセンサやロードセル + アームで同時に計測し, 比較することを推奨する.

使用例:
    python torque_test.py --torque 5          # +5 Nm で Ctrl+C まで押し続ける
    python torque_test.py --torque 5 --hold 10  # +5 Nm を 10 秒保持
    python torque_test.py --dir -1 --csv log.csv  # 負方向の最大トルクを目標, CSV に記録
"""

import argparse
import csv
import time

import serial

from servo_demo import SERIAL_PORT, BAUD_RATE, MotorProtocolSync

MODE_STOP = 0
MODE_CLOSED_LOOP = 1

# build_control_packet がクランプする outputTor の範囲 [Nm]
TOR_MAX_POS = 36.908174
TOR_MAX_NEG = -36.9093


def parse_args():
    p = argparse.ArgumentParser(description="最大トルク実測 (出力軸を固定して使う)")
    p.add_argument("--port", default=SERIAL_PORT, help="シリアルポート")
    p.add_argument("--id", type=int, default=0, help="サーボ ID (0-14)")
    p.add_argument("--torque", type=float, default=None,
                   help=f"目標トルク [Nm] (符号付き, {TOR_MAX_NEG}〜{TOR_MAX_POS}). "
                        "省略時はプロトコル上の最大トルク")
    p.add_argument("--dir", type=int, choices=(1, -1), default=1,
                   help="--torque 省略時の向き (+1 / -1)")
    p.add_argument("--ramp", type=float, default=2.0, help="トルクの立ち上げ/立ち下げ時間 [s]")
    p.add_argument("--hold", type=float, default=None,
                   help="目標トルクの保持時間 [s] (省略 / 0 以下で Ctrl+C まで保持し続ける)")
    p.add_argument("--max-speed", type=float, default=0.5,
                   help="この速度 [rad/s] を超えたら出力軸が固定されていないとみなし停止")
    p.add_argument("--max-move", type=float, default=0.2,
                   help="開始位置からこの量 [rad] 動いたら停止")
    p.add_argument("--max-temp", type=int, default=80, help="巻線温度の上限 [℃]")
    p.add_argument("--rate", type=float, default=200.0, help="制御周期 [Hz]")
    p.add_argument("--print-rate", type=float, default=5.0, help="状態表示の頻度 [Hz]")
    p.add_argument("--csv", default=None, help="全サンプルを書き出す CSV ファイル")
    p.add_argument("--allow-motion", action="store_true",
                   help="出力軸が動いても中断しない (--max-speed / --max-move を無効化)")
    p.add_argument("--kd", type=float, default=0.0,
                   help="速度ブレーキ Kd (0-21.285). 自由回転時の速度 ≈ |目標トルク| / Kd")
    p.add_argument("--yes", action="store_true", help="開始前の確認を省略する")
    return p.parse_args()


class Profile:
    """台形プロファイル: 0 → target (ramp 秒) → 保持 → 0 (ramp 秒).

    hold が None なら release() が呼ばれるまで保持し続ける.
    release() 以降は, その時点の指令値から ramp 秒で 0 へ下げる.
    """

    def __init__(self, target, ramp, hold):
        self.target = target
        self.ramp = ramp
        self.hold = hold
        self.release_t = None
        self.release_level = 0.0
        self.last = 0.0

    def release(self, t):
        if self.release_t is None:
            self.release_t = t
            self.release_level = self.last

    def phase(self, t):
        if self.release_t is None and self.hold is not None and t >= self.ramp + self.hold:
            self.release_t = self.ramp + self.hold
            self.release_level = self.target
        if self.release_t is not None:
            return "down"
        return "up" if t < self.ramp else "hold"

    def command(self, t):
        """指令トルクと区間名を返す. 終了後は (None, None)."""
        ph = self.phase(t)
        if ph == "up":
            cmd = self.target * t / self.ramp
        elif ph == "hold":
            cmd = self.target
        else:
            k = 1.0 - (t - self.release_t) / self.ramp
            if k <= 0.0:
                return None, None
            cmd = self.release_level * k
        self.last = cmd
        return cmd, ph


def mean(values):
    return sum(values) / len(values)


def main():
    args = parse_args()
    if args.torque is None:
        target = TOR_MAX_POS if args.dir > 0 else TOR_MAX_NEG
    else:
        target = args.torque
        if not TOR_MAX_NEG <= target <= TOR_MAX_POS:
            raise SystemExit(f"--torque は {TOR_MAX_NEG}〜{TOR_MAX_POS} Nm で指定してください")
    if args.ramp <= 0.0 or args.rate <= 0.0:
        raise SystemExit("--ramp と --rate は正の値を指定してください")
    hold = args.hold if args.hold is not None and args.hold > 0.0 else None

    hold_s = f"{hold} s" if hold is not None else "Ctrl+C まで"
    print(f"目標トルク {target:+.3f} Nm (立ち上げ {args.ramp} s / 保持 {hold_s})")
    if args.allow_motion:
        print("--allow-motion: 出力軸が動いても中断しません.")
        if args.kd > 0.0:
            print(f"自由回転時の速度は約 {abs(target) / args.kd:.1f} rad/s で頭打ちになります (Kd={args.kd})")
        else:
            print("[警告] --kd が 0 のため, 出力軸が自由なら摩擦に勝った分だけ加速し続けます")
    else:
        print("出力軸がトルクに耐えるよう確実に固定されていることを確認してください.")
    if not args.yes:
        input("準備ができたら Enter (中止は Ctrl+C) > ")

    motor = None
    rows = []
    reason = "完了"
    profile = Profile(target, args.ramp, hold)
    try:
        motor = MotorProtocolSync(args.port, BAUD_RATE, timeout=0.2)
        fb = motor.send_and_receive(args.id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if fb is None:
            raise SystemExit("サーボから応答がありません. 配線・ID・ポートを確認してください")
        if fb["MError"] != 0:
            raise SystemExit(f"MError={fb['MError']} のため開始しません")
        start_pos = fb["outputPos"]
        temp0 = fb["sensor"]
        print(f"開始位置 {start_pos:+.4f} rad, 電圧 {fb['vol']:.1f} V, 巻線温度 {temp0} ℃")
        print("Ctrl+C: トルクを徐々に下げて終了 / もう一度 Ctrl+C: 即停止")

        period = 1.0 / args.rate
        print_every = max(1, int(round(args.rate / args.print_rate)))
        t0 = time.perf_counter()
        next_tick = t0
        n = 0
        while True:
            try:
                t = time.perf_counter() - t0
                cmd, ph = profile.command(t)
                if cmd is None:
                    break

                fb = motor.send_and_receive(args.id, MODE_CLOSED_LOOP, 1,
                                            cmd, 0.0, 0.0, 0.0, args.kd)
                if fb is not None:
                    move = fb["outputPos"] - start_pos
                    rows.append((t, ph, cmd, fb["outputTor"], fb["outputSpd"], move,
                                 fb["vol"], fb["Temp"], fb["sensor"], fb["MError"]))
                    if n % print_every == 0:
                        print(f"t={t:6.2f}s [{ph:4s}]  cmd={cmd:+7.3f}  "
                              f"tor={fb['outputTor']:+7.3f} Nm  spd={fb['outputSpd']:+.3f}  "
                              f"move={move:+.4f}  vol={fb['vol']:.1f} V  temp={fb['sensor']} ℃")
                    if not args.allow_motion:
                        if abs(fb["outputSpd"]) > args.max_speed:
                            reason = (f"中断: 速度 {fb['outputSpd']:+.3f} rad/s > --max-speed "
                                      "(出力軸が固定されていない / 滑った)")
                            break
                        if abs(move) > args.max_move:
                            reason = (f"中断: 移動量 {move:+.4f} rad > --max-move "
                                      "(出力軸が固定されていない / 滑った)")
                            break
                    if fb["sensor"] > args.max_temp:
                        reason = f"中断: 巻線温度 {fb['sensor']} ℃ > --max-temp"
                        break
                    if fb["MError"] != 0:
                        reason = f"中断: MError={fb['MError']}"
                        break
                n += 1

                next_tick += period
                sleep = next_tick - time.perf_counter()
                if sleep > 0.0:
                    time.sleep(sleep)
                else:
                    next_tick = time.perf_counter()
            except KeyboardInterrupt:
                if profile.release_t is not None:
                    raise  # 2 回目 (立ち下げ中) の Ctrl+C は即停止
                profile.release(time.perf_counter() - t0)
                reason = "ユーザ終了 (立ち下げ後に停止)"
                print("\nトルクを下げています... (もう一度 Ctrl+C で即停止)")
    except serial.SerialException as e:
        reason = f"シリアルエラー: {e}"
    except KeyboardInterrupt:
        reason = "ユーザ中断 (即停止)"
    finally:
        if motor:
            try:
                motor.send_and_receive(args.id, MODE_STOP, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
                print("サーボ停止 (mode=0)")
            finally:
                motor.close()

    if args.csv and rows:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "tor_cmd", "tor_fb", "spd", "move", "vol",
                        "temp_shell", "temp_winding", "MError"])
            w.writerows(rows)
        print(f"{len(rows)} サンプルを {args.csv} に保存")

    print(f"\n==== 結果 ({reason}) ====")
    if not rows:
        print("フィードバックがありません")
        return
    sign = 1.0 if target >= 0 else -1.0
    peak = max(rows, key=lambda r: sign * r[3])
    print(f"目標トルク          {target:+8.3f} Nm")
    print(f"到達した最大指令    {max(sign * r[2] for r in rows) * sign:+8.3f} Nm")
    print(f"フィードバック最大  {peak[3]:+8.3f} Nm  (t={peak[0]:.2f} s, 指令 {peak[2]:+.3f} Nm)")
    held = [r for r in rows if r[1] == "hold"]
    if held:
        dur = held[-1][0] - held[0][0]
        m = mean([r[3] for r in held])
        print(f"保持時間            {dur:8.2f} s")
        print(f"保持区間の平均      {m:+8.3f} Nm  (指令比 {m / target * 100:.1f} %)")
        if dur >= 2.0:
            # 発熱による低下を見るため, 保持の最初と最後の 1 秒を比べる
            first = mean([r[3] for r in held if r[0] < held[0][0] + 1.0])
            last = mean([r[3] for r in held if r[0] > held[-1][0] - 1.0])
            print(f"保持 最初1s / 最後1s {first:+.3f} → {last:+.3f} Nm "
                  f"({(last - first) / first * 100:+.1f} %)")
    print(f"最大速度 / 最大移動 {max(abs(r[4]) for r in rows):.3f} rad/s / "
          f"{max(abs(r[5]) for r in rows):.4f} rad")
    print(f"電圧                {max(r[6] for r in rows):.1f} → 最低 {min(r[6] for r in rows):.1f} V")
    print(f"巻線温度            {temp0} → {rows[-1][8]} ℃ (最高 {max(r[8] for r in rows)} ℃)")


if __name__ == "__main__":
    main()

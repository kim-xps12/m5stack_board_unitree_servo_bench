// 位置制御モードで sin 波に沿ってサーボをスイングさせる AtomS3R 版 (python/sin_swing_demo.py の移植).
//
// パケットの組み立て/解析は stm32/firmware/App/protocol.c (無修正) を使い, mode=1
// (hybrid closed-loop) で以下の目標値を一定周期で送り続ける.
//
//     pos_des(t) = center + A * env(t) * sin(2π f t)
//     spd_des(t) = A * env(t) * 2π f * cos(2π f t)   (速度フィードフォワード)
//
// 開始時の出力軸位置を中心 (center) とし, 振幅は ramp 秒かけて 0 から立ち上げるので,
// 開始時に目標値が跳ぶことはない. timeout=1 (超時保護有効) で送信するため, 送信が止まれば
// サーボも約 1 s で停止する. 終了時 (stop コマンド・ボタン含む) は mode=0 を送って停止する.
//
// 配線: Grove (G2 -> 基板 J2 の TX, G1 <- 基板 J2 の RX). サーボは 4 Mbps 設定, こちらも 4 Mbps (BAUD_RATE).
// 操作 (USB シリアル):
//   start [amp=0.5] [freq=0.5] [kp=10] [kd=0.3] [duration=10] [ramp=1] [rate=200] [print=10] [id=0]
//         [baud=4000000]
//   stop
//   status    mode=0 で 1 回通信して状態を表示 (サーボは動かない)
// 前面ボタン: 停止中なら前回の条件で開始, スイング中なら停止.

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <math.h>

extern "C" {
#include "protocol.h"
}

static const gpio_num_t PIN_TX = GPIO_NUM_2;       // Grove -> 基板 J2 の TX
static const gpio_num_t PIN_RX = GPIO_NUM_1;       // Grove <- 基板 J2 の RX
static const gpio_num_t PIN_BUTTON = GPIO_NUM_41;  // 前面ボタン (押すと Low)
static const uart_port_t SERVO_UART = UART_NUM_1;
// Unitree 用の基板では 4 Mbps ちょうどで取りこぼしがない (docs/ttl_board_redesign.md §7.1).
// 旧基板 (DYNAMIXEL 用) では公称 4 Mbps がサーボの読める範囲の速い側の限界に近く, スイング中に約半数が
// 無応答になるので, start に baud=3950617 (80 MHz / (20 + 4/16), 範囲の中心寄り) を付ける (同 §8).
static const uint32_t BAUD_RATE = 4000000;
static const size_t CMD_LEN = sizeof(ControlData_t);   // 20
static const size_t FBK_LEN = sizeof(RecvData_t) - 2;  // 26
static const int64_t REPLY_TIMEOUT_US = 1000;

static const unsigned short MODE_STOP = 0;
static const unsigned short MODE_CLOSED_LOOP = 1;

struct Params {
  int id = 0;
  float amp = 0.5f;          // 振幅 [rad] (出力軸)
  float freq = 0.5f;         // 周波数 [Hz]
  float kp = 10.0f;          // 剛性ゲイン Kp (0-2128.523)
  float kd = 0.3f;           // 減衰ゲイン Kd (0-21.285)
  float duration = 10.0f;    // スイング時間 [s] (0 以下で stop まで継続)
  float ramp = 1.0f;         // 振幅の立ち上げ/立ち下げ時間 [s]
  float rate = 200.0f;       // 制御周期 [Hz]
  float print_rate = 10.0f;  // 状態表示の頻度 [Hz]
  uint32_t baud = BAUD_RATE; // 通信速度 [bps]
};

// 制御タスクから表示 (loop) へ渡す状態. 表示が詰まっても制御周期を乱さないようキューで渡す.
struct Sample {
  float t, des, pos, tor;
  uint32_t merr;
};

static const EventBits_t EV_START = BIT0;
static const EventBits_t EV_STOP = BIT1;

static Params run_params;  // 実行中/前回の条件 (ボタンでの開始に再利用)
static volatile bool running = false;
static TaskHandle_t control_task;
static EventGroupHandle_t events;
static QueueHandle_t samples;
static esp_timer_handle_t tick_timer;

static MotorCmd_t make_cmd(int id, unsigned short mode, unsigned short timeout, float tor, float spd,
                           float pos, float kp, float kd) {
  MotorCmd_t cmd = {};
  cmd.id = id;
  cmd.mode = mode;
  cmd.timeout = timeout;
  cmd.outputTor = tor;
  cmd.outputSpd = spd;
  cmd.outputPos = pos;
  cmd.Kp = kp;
  cmd.Kd = kd;
  return cmd;
}

enum Reply { REPLY_OK, REPLY_NONE, REPLY_BAD };  // 正常 / 無応答 / 受信したが読めない

// cmd を送って反馈 26 バイトを待つ. CRC まで正しい応答を data に読めたら REPLY_OK.
static Reply exchange(MotorCmd_t &cmd, MotorData_t &data) {
  modify_data(&cmd);
  uart_flush_input(SERVO_UART);
  uart_write_bytes(SERVO_UART, (const char *)&cmd.motor_send_data, CMD_LEN);
  uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(2));

  uint8_t rx[2 * FBK_LEN];
  size_t got = 0;
  int64_t deadline = esp_timer_get_time() + REPLY_TIMEOUT_US;
  while (got < FBK_LEN && esp_timer_get_time() < deadline) {
    int n = uart_read_bytes(SERVO_UART, rx + got, sizeof(rx) - got, 0);
    if (n > 0) got += n;
  }
  for (size_t k = 0; k + FBK_LEN <= got; k++) {
    if (rx[k] == 0xFC && rx[k + 1] == 0xEE) {
      memcpy(data.motor_recv_data.head, rx + k, FBK_LEN);
      data.rxlen = FBK_LEN;
      extract_data(&data);
      return data.correct ? REPLY_OK : REPLY_BAD;
    }
  }
  return got ? REPLY_BAD : REPLY_NONE;
}

static bool send_and_receive(MotorCmd_t &cmd, MotorData_t &data) { return exchange(cmd, data) == REPLY_OK; }

static void stop_servo(int id) {
  MotorCmd_t cmd = make_cmd(id, MODE_STOP, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  MotorData_t data = {};
  send_and_receive(cmd, data);
}

// 振幅エンベロープ (0→1→0). 立ち上げ・立ち下げは cos 補間で滑らかにする.
static float envelope(float t, float duration, float ramp) {
  if (ramp <= 0.0f) return 1.0f;
  float k = fminf(1.0f, t / ramp);
  if (duration > 0.0f) k = fminf(k, fmaxf(0.0f, (duration - t) / ramp));
  return 0.5f - 0.5f * cosf((float)M_PI * k);
}

static void on_tick(void *) { xTaskNotifyGive(control_task); }

static void control_loop(void *) {
  static MotorData_t fb;
  for (;;) {
    xEventGroupWaitBits(events, EV_START, pdTRUE, pdFALSE, portMAX_DELAY);
    const Params p = run_params;
    uart_set_baudrate(SERVO_UART, p.baud);

    // 現在位置を取得してスイング中心にする (mode=0: 停止状態のまま読み出し)
    MotorCmd_t cmd = make_cmd(p.id, MODE_STOP, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (!send_and_receive(cmd, fb)) {
      Serial.println("サーボから応答がありません. 配線・ID を確認してください");
      running = false;
      continue;
    }
    const float center = fb.outputPos;
    Serial.printf("中心位置 = %.4f rad, 電圧 = %.1f V, 温度 = %d ℃\n", center, fb.vol, fb.Temp);
    Serial.printf("スイング開始: A=%g rad, f=%g Hz, Kp=%g, Kd=%g  (stop またはボタンで停止)\n", p.amp,
                  p.freq, p.kp, p.kd);

    const float omega = 2.0f * (float)M_PI * p.freq;
    const int print_every = (int)fmaxf(1.0f, roundf(p.rate / p.print_rate));
    // duration 指定時は立ち下げ分まで含めてその時間で終える
    const float end_time = p.duration > 0.0f ? p.duration : INFINITY;
    int n = 0, none = 0, bad = 0;
    bool interrupted = false;

    ulTaskNotifyTake(pdTRUE, 0);
    const int64_t t0 = esp_timer_get_time();
    esp_timer_start_periodic(tick_timer, (uint64_t)(1e6f / p.rate));
    for (;;) {
      float t = (esp_timer_get_time() - t0) / 1e6f;
      if (t >= end_time) break;
      if (xEventGroupGetBits(events) & EV_STOP) {
        interrupted = true;
        break;
      }

      float a = p.amp * envelope(t, p.duration, p.ramp);
      float pos_des = center + a * sinf(omega * t);
      float spd_des = a * omega * cosf(omega * t);

      cmd = make_cmd(p.id, MODE_CLOSED_LOOP, 1, 0.0f, spd_des, pos_des, p.kp, p.kd);
      Reply r = exchange(cmd, fb);
      if (r == REPLY_OK && n % print_every == 0) {
        Sample s = {t, pos_des, fb.outputPos, fb.outputTor, fb.MError};
        xQueueSend(samples, &s, 0);
      }
      none += r == REPLY_NONE;
      bad += r == REPLY_BAD;
      n++;

      // 固定周期で送信 (遅れた場合は取り戻さずに次周期へ)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    esp_timer_stop(tick_timer);
    float elapsed = (esp_timer_get_time() - t0) / 1e6f;

    stop_servo(p.id);
    for (int i = 0; i < 50 && uxQueueMessagesWaiting(samples); i++) vTaskDelay(pdMS_TO_TICKS(2));
    if (interrupted) {
      Serial.println("ユーザ中断");
    } else {
      Serial.printf("完了: %d 周期 / %.2f s (平均 %.1f Hz)\n", n, elapsed, n / elapsed);
    }
    Serial.printf("通信失敗: 無応答 %d, 受信エラー %d / %d 周期 (%u bps)\n", none, bad, n, (unsigned)p.baud);
    Serial.println("サーボ停止 (mode=0)");
    running = false;
  }
}

// 既定値 (Python 版と同じ) に key=value を上書きする.
static bool parse_params(char *args, Params &p) {
  p = Params();
  char *save = NULL;
  for (char *tok = strtok_r(args, " ", &save); tok; tok = strtok_r(NULL, " ", &save)) {
    char *eq = strchr(tok, '=');
    if (!eq) {
      Serial.printf("key=value の形で指定してください: %s\n", tok);
      return false;
    }
    *eq = 0;
    float v = atof(eq + 1);
    if (!strcmp(tok, "id")) p.id = (int)v;
    else if (!strcmp(tok, "amp")) p.amp = v;
    else if (!strcmp(tok, "freq")) p.freq = v;
    else if (!strcmp(tok, "kp")) p.kp = v;
    else if (!strcmp(tok, "kd")) p.kd = v;
    else if (!strcmp(tok, "duration")) p.duration = v;
    else if (!strcmp(tok, "ramp")) p.ramp = v;
    else if (!strcmp(tok, "rate")) p.rate = v;
    else if (!strcmp(tok, "print")) p.print_rate = v;
    else if (!strcmp(tok, "baud")) p.baud = (uint32_t)v;
    else {
      Serial.printf("不明なパラメータ: %s\n", tok);
      return false;
    }
  }
  if (p.freq <= 0.0f || p.rate <= 0.0f || p.print_rate <= 0.0f) {
    Serial.println("freq, rate, print は正の値を指定してください");
    return false;
  }
  if (p.rate > 1000.0f) {
    Serial.println("rate は 1000 Hz 以下を指定してください");
    return false;
  }
  if (p.id < 0 || p.id > 14) {
    Serial.println("id は 0-14 を指定してください");
    return false;
  }
  return true;
}

static void start_swing() {
  running = true;
  xEventGroupClearBits(events, EV_STOP);
  xEventGroupSetBits(events, EV_START);
}

static void print_status() {
  MotorCmd_t cmd = make_cmd(run_params.id, MODE_STOP, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  MotorData_t fb = {};
  if (send_and_receive(cmd, fb)) {
    Serial.printf("id=%u mode=%u pos=%.4f rad, 電圧 = %.1f V, 温度 = %d ℃, MError=%u\n", fb.motor_id, fb.mode,
                  fb.outputPos, fb.vol, fb.Temp, (unsigned)fb.MError);
  } else {
    Serial.println("サーボから応答がありません. 配線・ID を確認してください");
  }
}

static void handle_line(char *line) {
  char *save = NULL;
  char *op = strtok_r(line, " ", &save);
  if (!op) return;
  if (!strcmp(op, "start")) {
    Params p;
    char none[1] = {0};
    if (running) Serial.println("スイング中です (stop で停止)");
    else if (parse_params(save ? save : none, p)) {
      run_params = p;
      start_swing();
    }
  } else if (!strcmp(op, "stop")) {
    if (running) xEventGroupSetBits(events, EV_STOP);
  } else if (!strcmp(op, "status")) {
    if (running) Serial.println("スイング中です");
    else print_status();
  } else {
    Serial.println("コマンド: start [amp= freq= kp= kd= duration= ramp= rate= print= id=] / stop / status");
  }
}

void setup() {
  Serial.setTxTimeoutMs(0);  // PC が読んでいなくても表示で止まらない
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  uart_config_t u = {};
  u.baud_rate = BAUD_RATE;
  u.data_bits = UART_DATA_8_BITS;
  u.parity = UART_PARITY_DISABLE;
  u.stop_bits = UART_STOP_BITS_1;
  u.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  u.source_clk = UART_SCLK_APB;
  ESP_ERROR_CHECK(uart_param_config(SERVO_UART, &u));
  ESP_ERROR_CHECK(uart_driver_install(SERVO_UART, 1024, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(SERVO_UART, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  uart_set_rx_timeout(SERVO_UART, 2);

  events = xEventGroupCreate();
  samples = xQueueCreate(32, sizeof(Sample));
  esp_timer_create_args_t targs = {};
  targs.callback = on_tick;
  targs.name = "swing";
  ESP_ERROR_CHECK(esp_timer_create(&targs, &tick_timer));
  xTaskCreatePinnedToCore(control_loop, "swing", 4096, NULL, 10, &control_task, 1);

  Serial.printf("sin_swing: %u bps, TX=G2 RX=G1\n", (unsigned)BAUD_RATE);
  Serial.println("コマンド: start [amp= freq= kp= kd= duration= ramp= rate= print= id=] / stop / status");
}

void loop() {
  static char line[256];
  static size_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[len] = 0;
      handle_line(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }

  // 前面ボタン: 押した瞬間 (High -> Low) だけ反応させ, 50 ms はチャタリングとして無視する
  static int last = HIGH;
  static uint32_t last_change = 0;
  int level = digitalRead(PIN_BUTTON);
  if (level != last && millis() - last_change > 50) {
    last_change = millis();
    last = level;
    if (level == LOW) {
      if (running) xEventGroupSetBits(events, EV_STOP);
      else start_swing();
    }
  }

  Sample s;
  while (xQueueReceive(samples, &s, 0) == pdTRUE) {
    Serial.printf("t=%6.2fs  des=%+.4f  pos=%+.4f  err=%+.4f rad  tor=%+.3f Nm  MError=%u\n", s.t, s.des,
                  s.pos, s.des - s.pos, s.tor, (unsigned)s.merr);
  }
  delay(1);
}

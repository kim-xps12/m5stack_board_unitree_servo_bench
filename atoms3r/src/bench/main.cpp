// AtomS3R bench firmware for the J288/S288 link through the DYNAMIXEL TTL board
// (MAX3485 with transistor auto-direction). Driven over USB serial by tools/bench.py.
//
//   G2 (Grove)        -> board STS_TX
//   G1 (Grove)        <- board STS_RX (MAX3485 RO)
//   (checked on hardware: with a straight Grove cable the board's J2 TX/RX land on G2/G1)
//   G5 (bottom pin)   <- board A line; bench tests only, with the servo and J1 power unplugged
//
// The RMT peripheral serves both as a pattern generator on the TX pin (when the UART is not
// driving it) and as a one-pin logic analyser with 12.5 ns resolution on the RX pin or G5.
//
// Commands (one per line, every reply ends with "#END"):
//   PING
//   LEVELS
//   PULL <R|A> <u|d|n>
//   HOLD <0|1|z> [u|d|n] drive the TX pin statically, or release it as an input;
//                        the next GEN/UTX/XCHG takes it back
//   GEN  <R|A> <clkdiv> <filt> <idle> <reps> <pull> <n> <low0> <high0> <low1> ...
//   UTX  <R|A> <clkdiv> <filt> <idle> <reps> <pull> <baud> <hex>
//   XCHG <baud> <n> <id> <pull> <period_us> <capevery> <clkdiv> <filt> <idle> <dumpevery> <pattern>
//        pattern 0: stop command (mode 0, mostly zero bytes)
//        pattern 1: varying payload without torque (mode 1, Kp = Kd = torque = 0, random targets)
// Durations for GEN are in 12.5 ns ticks. <filt> is in 12.5 ns ticks, <idle> in capture ticks
// (12.5 ns * clkdiv). <pull> applies to the captured pin (XCHG: to the RX pin).
// Capture pin T records the TX pin itself (RMT/UART output looped back through the pad).

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <freertos/ringbuf.h>
#include <soc/gpio_periph.h>
#include <soc/io_mux_reg.h>
#include <soc/rmt_periph.h>

extern "C" {
#include "protocol.h"
}

static const gpio_num_t PIN_TX = GPIO_NUM_2;
static const gpio_num_t PIN_RX = GPIO_NUM_1;
static const gpio_num_t PIN_PROBE = GPIO_NUM_5;

static const uart_port_t SERVO_UART = UART_NUM_1;
static const rmt_channel_t GEN_CH = RMT_CHANNEL_0;  // TX, memory blocks 0-3
static const rmt_channel_t CAP_CH = RMT_CHANNEL_4;  // RX, memory blocks 4-7
static const int GEN_MEM_BLOCKS = 4;
static const int CAP_MEM_BLOCKS = 4;
static const size_t CAP_RINGBUF_SIZE = 16384;
static const size_t CMD_PACKET_LEN = sizeof(ControlData_t);         // 20
static const size_t FBK_PACKET_LEN = sizeof(RecvData_t) - 2;        // 26

static RingbufHandle_t cap_rb;
static rmt_item32_t gen_items[GEN_MEM_BLOCKS * RMT_MEM_ITEM_NUM * 2];

enum TxOwner { TX_NONE, TX_GEN, TX_UART };
static TxOwner tx_owner = TX_NONE;
static char rx_pull = 'u';

struct CapCfg {
  char pin;
  uint8_t clkdiv;
  uint8_t filt;
  uint16_t idle;
};

static gpio_num_t cap_gpio(char pin) { return pin == 'A' ? PIN_PROBE : pin == 'T' ? PIN_TX : PIN_RX; }

static void set_pull(gpio_num_t gpio, char mode) {
  if (gpio == PIN_TX) return;
  gpio_set_pull_mode(gpio, mode == 'u'   ? GPIO_PULLUP_ONLY
                           : mode == 'd' ? GPIO_PULLDOWN_ONLY
                                         : GPIO_FLOATING);
  if (gpio == PIN_RX) rx_pull = mode;
}

static void route_tx_to_gen() {
  if (tx_owner == TX_GEN) return;
  gpio_set_pull_mode(PIN_TX, GPIO_FLOATING);
  rmt_set_gpio(GEN_CH, RMT_MODE_TX, PIN_TX, false);
  tx_owner = TX_GEN;
}

static void route_tx_to_uart() {
  if (tx_owner == TX_UART) return;
  // uart_set_pin() forces a pull-up on RX; restore the requested pull afterwards.
  char keep = rx_pull;
  gpio_set_pull_mode(PIN_TX, GPIO_FLOATING);
  uart_set_pin(SERVO_UART, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  set_pull(PIN_RX, keep);
  tx_owner = TX_UART;
}

static uint32_t set_baud(uint32_t baud) {
  uart_set_baudrate(SERVO_UART, baud);
  uint32_t actual = 0;
  uart_get_baudrate(SERVO_UART, &actual);
  return actual;
}

// ---------------------------------------------------------------------------------------------
// Capture

static void cap_drain() {
  size_t size;
  void *p;
  while ((p = xRingbufferReceive(cap_rb, &size, 0)) != NULL) vRingbufferReturnItem(cap_rb, p);
}

static void cap_start(const CapCfg &c) {
  rmt_rx_stop(CAP_CH);
  if (c.pin == 'T') {
    // rmt_set_gpio() would turn the TX pin into an input; only tap its pad input instead.
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PIN_TX]);
    esp_rom_gpio_connect_in_signal(PIN_TX, rmt_periph_signals.groups[0].channels[CAP_CH].rx_sig, false);
  } else {
    rmt_set_gpio(CAP_CH, RMT_MODE_RX, cap_gpio(c.pin), false);
  }
  rmt_set_clk_div(CAP_CH, c.clkdiv);
  rmt_set_rx_filter(CAP_CH, c.filt > 0, c.filt);
  rmt_set_rx_idle_thresh(CAP_CH, c.idle);
  cap_drain();
  rmt_rx_start(CAP_CH, true);
}

// Waits for the idle threshold to end the frame, then prints "CAP <pin> <idx> <clkdiv> <runs>".
// Runs are "L<ticks>"/"H<ticks>"; separate frames (split by idle) are joined with " |".
static void cap_finish(const CapCfg &c, int idx) {
  delayMicroseconds((uint32_t)c.idle * c.clkdiv / 80 + 100);
  Serial.printf("CAP %c %d %u", c.pin, idx, c.clkdiv);
  size_t size;
  rmt_item32_t *items;
  bool first = true;
  while ((items = (rmt_item32_t *)xRingbufferReceive(cap_rb, &size, pdMS_TO_TICKS(first ? 5 : 1))) != NULL) {
    if (!first) Serial.print(" |");
    first = false;
    size_t n = size / sizeof(rmt_item32_t);
    for (size_t i = 0; i < n; i++) {
      if (!items[i].duration0) break;
      Serial.printf(" %c%u", items[i].level0 ? 'H' : 'L', items[i].duration0);
      if (!items[i].duration1) break;
      Serial.printf(" %c%u", items[i].level1 ? 'H' : 'L', items[i].duration1);
    }
    vRingbufferReturnItem(cap_rb, items);
  }
  Serial.println();
  rmt_rx_stop(CAP_CH);
}

// ---------------------------------------------------------------------------------------------
// Helpers

static void print_hex(const char *tag, int idx, const uint8_t *buf, size_t len) {
  Serial.printf("%s %d ", tag, idx);
  for (size_t i = 0; i < len; i++) Serial.printf("%02X", buf[i]);
  Serial.println();
}

static size_t parse_hex(const char *s, uint8_t *out, size_t max) {
  size_t n = 0;
  while (s[0] && s[1] && n < max) {
    char b[3] = {s[0], s[1], 0};
    out[n++] = (uint8_t)strtoul(b, NULL, 16);
    s += 2;
  }
  return n;
}

static bool parse_cap(char **save, CapCfg &c) {
  char *pin = strtok_r(NULL, " ", save);
  char *div = strtok_r(NULL, " ", save);
  char *filt = strtok_r(NULL, " ", save);
  char *idle = strtok_r(NULL, " ", save);
  if (!pin || !div || !filt || !idle) return false;
  c.pin = pin[0] == 'A' || pin[0] == 'T' ? pin[0] : 'R';
  c.clkdiv = (uint8_t)atoi(div);
  c.filt = (uint8_t)atoi(filt);
  c.idle = (uint16_t)atoi(idle);
  return c.clkdiv > 0 && c.idle > 0;
}

// ---------------------------------------------------------------------------------------------
// Commands

static void cmd_gen(char **save) {
  CapCfg c;
  if (!parse_cap(save, c)) return (void)Serial.println("ERR args");
  char *s;
  int reps = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 1;
  char pull = (s = strtok_r(NULL, " ", save)) ? s[0] : 'n';
  int n = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 0;
  const int max_durs = (int)(sizeof(gen_items) / sizeof(gen_items[0])) * 2;
  if (n <= 0 || n > max_durs) return (void)Serial.println("ERR n");

  memset(gen_items, 0, sizeof(gen_items));
  for (int i = 0; i < n; i++) {
    if (!(s = strtok_r(NULL, " ", save))) return (void)Serial.println("ERR durations");
    uint32_t d = strtoul(s, NULL, 10);
    if (d == 0 || d > 32767) return (void)Serial.println("ERR duration range");
    rmt_item32_t &it = gen_items[i / 2];
    if (i % 2 == 0) {
      it.level0 = 0;
      it.duration0 = d;
    } else {
      it.level1 = 1;
      it.duration1 = d;
    }
  }
  int items = (n + 1) / 2;
  if (n % 2) gen_items[items - 1].level1 = 1;  // trailing zero duration ends the burst high

  route_tx_to_gen();
  set_pull(cap_gpio(c.pin), pull);
  for (int r = 0; r < reps; r++) {
    cap_start(c);
    delayMicroseconds(20);
    rmt_write_items(GEN_CH, gen_items, items, true);
    cap_finish(c, r);
    delay(2);
  }
}

static void cmd_utx(char **save) {
  CapCfg c;
  if (!parse_cap(save, c)) return (void)Serial.println("ERR args");
  char *s;
  int reps = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 1;
  char pull = (s = strtok_r(NULL, " ", save)) ? s[0] : 'n';
  uint32_t baud = (s = strtok_r(NULL, " ", save)) ? strtoul(s, NULL, 10) : 4000000;
  if (!(s = strtok_r(NULL, " ", save))) return (void)Serial.println("ERR hex");
  static uint8_t tx[256];
  size_t len = parse_hex(s, tx, sizeof(tx));

  route_tx_to_uart();
  set_pull(cap_gpio(c.pin), pull);
  Serial.printf("BAUD %u\n", set_baud(baud));
  print_hex("TXB", 0, tx, len);
  for (int r = 0; r < reps; r++) {
    uart_flush_input(SERVO_UART);
    cap_start(c);
    delayMicroseconds(20);
    uart_write_bytes(SERVO_UART, (const char *)tx, len);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
    cap_finish(c, r);
    uint8_t rx[256];
    int got = uart_read_bytes(SERVO_UART, rx, sizeof(rx), 0);
    if (got > 0) print_hex("RXB", r, rx, got);
    delay(2);
  }
}

static void cmd_xchg(char **save) {
  char *s;
  uint32_t baud = (s = strtok_r(NULL, " ", save)) ? strtoul(s, NULL, 10) : 4000000;
  int n = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 100;
  int id = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 0;
  char pull = (s = strtok_r(NULL, " ", save)) ? s[0] : 'u';
  int period_us = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 1000;
  int capevery = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 0;
  CapCfg c = {'R', 1, 4, 32000};
  if ((s = strtok_r(NULL, " ", save))) c.clkdiv = (uint8_t)atoi(s);
  if ((s = strtok_r(NULL, " ", save))) c.filt = (uint8_t)atoi(s);
  if ((s = strtok_r(NULL, " ", save))) c.idle = (uint16_t)atoi(s);
  int dumpevery = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 0;
  int pattern = (s = strtok_r(NULL, " ", save)) ? atoi(s) : 0;

  // The command is built by the unmodified STM32 protocol code.
  static MotorCmd_t cmd;
  static MotorData_t data;
  memset(&cmd, 0, sizeof(cmd));
  cmd.id = id;
  cmd.mode = 0;
  cmd.timeout = 0;
  modify_data(&cmd);
  const uint8_t *tx = (const uint8_t *)&cmd.motor_send_data;

  route_tx_to_uart();
  set_pull(PIN_RX, pull);
  uint32_t actual = set_baud(baud);
  print_hex("TXB", 0, tx, CMD_PACKET_LEN);

  int ok = 0, noresp = 0, nohdr = 0, shortpk = 0, crc = 0;
  int64_t lat_min = INT64_MAX, lat_max = 0, lat_sum = 0;
  int lat_n = 0;
  bool have_fbk = false;
  uint8_t rx[256];

  for (int i = 0; i < n; i++) {
    int64_t t0 = esp_timer_get_time();
    if (pattern == 1) {
      // Zero gains and torque, so the targets only vary the bytes on the wire.
      cmd.mode = 1;
      cmd.timeout = 1;
      cmd.outputPos = (int32_t)esp_random() / 2147483648.0f * 100.0f;
      cmd.outputSpd = (int32_t)esp_random() / 2147483648.0f * 50.0f;
      modify_data(&cmd);
    }
    bool cap = capevery > 0 && i % capevery == 0;
    uart_flush_input(SERVO_UART);
    if (cap) cap_start(c);
    uart_write_bytes(SERVO_UART, (const char *)tx, CMD_PACKET_LEN);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
    int64_t t_tx = esp_timer_get_time();

    // Wait until a full reply has been buffered and the line has gone quiet, or 3 ms.
    size_t len = 0, last = 0;
    int64_t t_first = -1, t_change = t_tx;
    while (true) {
      int64_t now = esp_timer_get_time();
      uart_get_buffered_data_len(SERVO_UART, &len);
      if (len != last) {
        if (t_first < 0) t_first = now;
        last = len;
        t_change = now;
      }
      if (len >= FBK_PACKET_LEN && now - t_change > 50) break;
      if (now - t_tx > 3000) break;
    }
    int got = len ? uart_read_bytes(SERVO_UART, rx, len < sizeof(rx) ? len : sizeof(rx), 0) : 0;
    if (got < 0) got = 0;
    if (cap) cap_finish(c, i);
    if (dumpevery > 0 && i % dumpevery == 0) print_hex("RXB", i, rx, got);

    int start = -1;
    for (int k = 0; k + 1 < got; k++) {
      if (rx[k] == 0xFC && rx[k + 1] == 0xEE) {
        start = k;
        break;
      }
    }
    if (got == 0) {
      noresp++;
    } else if (start < 0) {
      nohdr++;
    } else if (got - start < (int)FBK_PACKET_LEN) {
      shortpk++;
    } else {
      memcpy(data.motor_recv_data.head, rx + start, FBK_PACKET_LEN);
      data.rxlen = FBK_PACKET_LEN;
      extract_data(&data);
      if (data.correct) {
        ok++;
        have_fbk = true;
      } else {
        crc++;
      }
    }
    if (t_first >= 0) {
      int64_t lat = t_first - t_tx;
      lat_min = lat < lat_min ? lat : lat_min;
      lat_max = lat > lat_max ? lat : lat_max;
      lat_sum += lat;
      lat_n++;
    }
    while (esp_timer_get_time() - t0 < period_us) {
    }
    if (i % 200 == 199) delay(1);
  }
  if (pattern == 1) {
    memset(&cmd, 0, sizeof(cmd));
    cmd.id = id;
    modify_data(&cmd);
    uart_write_bytes(SERVO_UART, (const char *)tx, CMD_PACKET_LEN);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(20));
    delay(2);
    uart_flush_input(SERVO_UART);
  }

  Serial.printf("XRES baud=%u n=%d ok=%d noresp=%d nohdr=%d short=%d crc=%d lat_min=%lld lat_avg=%lld lat_max=%lld\n",
                actual, n, ok, noresp, nohdr, shortpk, crc, lat_n ? lat_min : -1LL,
                lat_n ? lat_sum / lat_n : -1LL, lat_n ? lat_max : -1LL);
  if (have_fbk) {
    Serial.printf("FBK id=%u mode=%u temp=%d winding=%u vol=%.1f pos=%.4f spd=%.4f tor=%.4f merr=%u warn=%u\n",
                  data.motor_id, data.mode, data.Temp, data.sensor, data.vol, data.outputPos,
                  data.outputSpd, data.outputTor, (unsigned)data.MError, data.MWarn);
  }
}

static void handle(char *line) {
  char *save = NULL;
  char *op = strtok_r(line, " ", &save);
  if (!op) return;
  if (!strcmp(op, "PING")) {
    uint32_t hz = 0;
    rmt_get_counter_clock(GEN_CH, &hz);
    Serial.printf("OK bench 1 gen_clk=%u\n", hz);
  } else if (!strcmp(op, "LEVELS")) {
    Serial.printf("LEVELS TX=%d RX=%d G5=%d\n", gpio_get_level(PIN_TX), gpio_get_level(PIN_RX),
                  gpio_get_level(PIN_PROBE));
  } else if (!strcmp(op, "READ")) {
    // READ <gpio> <u|d|n>: bottom-header pins only, to find where a probe wire is plugged in.
    char *g = strtok_r(NULL, " ", &save);
    char *pull = strtok_r(NULL, " ", &save);
    int n = g ? atoi(g) : -1;
    if (n != 5 && n != 6 && n != 7 && n != 8 && n != 38 && n != 39) {
      Serial.println("ERR pin");
    } else {
      gpio_num_t gpio = (gpio_num_t)n;
      gpio_set_direction(gpio, GPIO_MODE_INPUT);
      set_pull(gpio, pull ? pull[0] : 'n');
      delayMicroseconds(200);
      Serial.printf("READ G%d=%d\n", n, gpio_get_level(gpio));
    }
  } else if (!strcmp(op, "HOLD")) {
    char *v = strtok_r(NULL, " ", &save);
    char *pull = strtok_r(NULL, " ", &save);
    if (v && v[0] == 'z') {
      // Release the TX pin (input) with an optional pull, to probe what the board drives.
      gpio_set_direction(PIN_TX, GPIO_MODE_INPUT);
      gpio_set_pull_mode(PIN_TX, pull && pull[0] == 'u'   ? GPIO_PULLUP_ONLY
                                 : pull && pull[0] == 'd' ? GPIO_PULLDOWN_ONLY
                                                          : GPIO_FLOATING);
    } else {
      gpio_set_pull_mode(PIN_TX, GPIO_FLOATING);
      gpio_set_direction(PIN_TX, GPIO_MODE_INPUT_OUTPUT);
      esp_rom_gpio_connect_out_signal(PIN_TX, SIG_GPIO_OUT_IDX, false, false);
      gpio_set_level(PIN_TX, v && v[0] == '1');
    }
    tx_owner = TX_NONE;
    Serial.println("OK");
  } else if (!strcmp(op, "PULL")) {
    char *pin = strtok_r(NULL, " ", &save);
    char *mode = strtok_r(NULL, " ", &save);
    if (pin && mode) set_pull(cap_gpio(pin[0]), mode[0]);
    Serial.println(pin && mode ? "OK" : "ERR args");
  } else if (!strcmp(op, "GEN")) {
    cmd_gen(&save);
  } else if (!strcmp(op, "UTX")) {
    cmd_utx(&save);
  } else if (!strcmp(op, "XCHG")) {
    cmd_xchg(&save);
  } else {
    Serial.println("ERR unknown");
  }
  Serial.println("#END");
}

// ---------------------------------------------------------------------------------------------

void setup() {
  Serial.setRxBufferSize(4096);  // GEN lines exceed the default 256-byte buffer
  Serial.begin(115200);

  rmt_config_t g = {};
  g.rmt_mode = RMT_MODE_TX;
  g.channel = GEN_CH;
  g.gpio_num = PIN_TX;
  g.clk_div = 1;
  g.mem_block_num = GEN_MEM_BLOCKS;
  g.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
  g.tx_config.idle_output_en = true;
  ESP_ERROR_CHECK(rmt_config(&g));
  ESP_ERROR_CHECK(rmt_driver_install(GEN_CH, 0, 0));
  // The idle level only reaches the pad after a transmission; send an all-high item so the TX
  // pin (the board's STS_TX) idles high instead of holding the board's DE on.
  rmt_item32_t high = {};
  high.level0 = 1;
  high.duration0 = 8;
  high.level1 = 1;
  high.duration1 = 8;
  rmt_write_items(GEN_CH, &high, 1, true);
  tx_owner = TX_GEN;

  rmt_config_t r = {};
  r.rmt_mode = RMT_MODE_RX;
  r.channel = CAP_CH;
  r.gpio_num = PIN_RX;
  r.clk_div = 1;
  r.mem_block_num = CAP_MEM_BLOCKS;
  r.rx_config.idle_threshold = 32000;
  r.rx_config.filter_en = true;
  r.rx_config.filter_ticks_thresh = 4;
  ESP_ERROR_CHECK(rmt_config(&r));
  ESP_ERROR_CHECK(rmt_driver_install(CAP_CH, CAP_RINGBUF_SIZE, 0));
  ESP_ERROR_CHECK(rmt_get_ringbuf_handle(CAP_CH, &cap_rb));
  rmt_rx_stop(CAP_CH);

  uart_config_t u = {};
  u.baud_rate = 4000000;
  u.data_bits = UART_DATA_8_BITS;
  u.parity = UART_PARITY_DISABLE;
  u.stop_bits = UART_STOP_BITS_1;
  u.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  u.source_clk = UART_SCLK_APB;
  ESP_ERROR_CHECK(uart_param_config(SERVO_UART, &u));
  ESP_ERROR_CHECK(uart_driver_install(SERVO_UART, 2048, 0, 0, NULL, 0));
  uart_set_rx_timeout(SERVO_UART, 2);

  gpio_set_direction(PIN_PROBE, GPIO_MODE_INPUT);
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PIN_TX]);  // lets LEVELS read back the TX pin
  set_pull(PIN_RX, 'u');
  set_pull(PIN_PROBE, 'n');
  Serial.println("#READY bench 1");
}

void loop() {
  static char line[4096];
  static size_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[len] = 0;
      handle(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}

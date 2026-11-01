/**
 * pim.ino — 424 kHz リングバッファ版の本体。
 *   bring-up / 磁界 / RX / Transparent は firmware-multitag と同一。
 *   捕捉を「単発 13.1 ms」から「DMA 循環で回しっぱなし」に変え、2 段の追跡
 *   （1 段目: 活動判定 → 2 段目: セグメント切り出し + 復号）を足した。
 *   復号アルゴリズム（running-sum IQ・SOF 検出・Manchester・貪欲ペアリング）は
 *   multitag と同一で、読む配列を g_capture から g_seg に変えただけ。
 */

/* ===========================================================================
 *  bring-up / 磁界 / RX 設定 / Transparent（firmware-multitag と同一）
 * ==========================================================================*/
static uint16_t measureVoltage(uint8_t mpsv) {
  regChange(REG_REGULATOR_CONTROL, 0x06, mpsv);
  cmdExec(CMD_MEASURE_VDD); delayMicroseconds(300);
  uint16_t v = regRead(REG_AD_RESULT);
  return (uint16_t)(v * 23 + (v * 438 + 500) / 1000);
}

static void pim_apply_vddrf(void);   /* 定義は mlogf の後（下） */

bool pim_bringup() {
  cmdExec(CMD_SET_DEFAULT);
  regClr(REG_IO_CONF2, IO2_miso_pd1 | IO2_miso_pd2);
  regSet(REG_OP_CONTROL, OPC_en);
  uint32_t t0 = millis();
  while (!(regRead(REG_AUX_DISPLAY) & AUX_osc_ok) && (millis() - t0) < 20) { }
  uint16_t vdd = measureVoltage(0);
  regChange(REG_IO_CONF2, IO2_sup3V, (vdd < 3600) ? IO2_sup3V : 0);
  logf("[bringup] VDD=%umV -> sup3V=%d\r\n", vdd, (vdd < 3600));
  cmdExec(CMD_ADJUST_REGULATORS);
  /* ★自動調整の結果を «上書き» して VDD_RF を固定する（既定 5 V）。
     アンテナ校正はこの電圧で行いたいので CALIBRATE_ANTENNA の «前» に置く。 */
  pim_apply_vddrf();
  cmdExec(CMD_CALIBRATE_ANTENNA);

  uint8_t id = regRead(REG_IC_IDENTITY);
  bool ok = ((id & 0xF8) == 0x08);
  uint8_t aux = regRead(REG_AUX_DISPLAY);
  logf("[bringup] IC_IDENTITY(0x3F)=0x%02X chip %s (rev=0x%02X)\r\n", id, ok ? "OK" : "MISMATCH", id & 0x07);
  logf("[bringup] AUX_DISPLAY(0x30)=0x%02X osc_ok=%d\r\n", aux, !!(aux & AUX_osc_ok));
  if (!ok) logf("[bringup] *** SPI/chip not responding - check wiring ***\r\n");
  g_diag[0] = 0x50494D20u;
  g_diag[1] = ((uint32_t)ok << 24) | ((uint32_t)id << 16) | ((uint32_t)(id & 7) << 8)
              | ((uint32_t)!!(aux & AUX_osc_ok) << 1);
  return ok;
}

/* 磁界サイクル中の定型ログは黙らせる */
static uint8_t m_mute = 0;
#define mlogf(...) do { if (!m_mute) logf(__VA_ARGS__); } while (0)

/* Transparent 中に SPI.transfer() を呼ぶと STM32duino コアのビジーループで固まる。
 * レジスタに触るコマンドは必ず pim_need_spi() で先に SPI を戻すこと。 */
static uint8_t m_tpm = 0;

/* VDD_RF[mV] 換算（RFAL / host/pim_multitag.py の vddrf_mv と同一式） */
static uint16_t pim_vddrf_mv(uint8_t rege, bool sup3v) {
  if (sup3v) return (uint16_t)(2400 + 100 * ((rege > 4) ? (rege - 5) : 0));
  return (uint16_t)(3600 + 100 * rege);
}

/* ★VDD_RF を «手動固定» する（既定 = PIM_VDDRF_REGE = 5 V 設定）。
     REGULATOR_CONTROL(0x2A): bit7 = reg_s（1 = rege を使う手動モード）、[6:3] = rege。
     g_pim.vddrf_rege = 0xFF のときは «何も書かない» = 従来どおり
     ADJUST_REGULATORS(0xD6) が決めた自動値（REGULATOR_RESULT の reg_d）が効く。
     ★SET_DEFAULT で消えるので、Transparent に入るたびに呼ぶこと。 */
static void pim_apply_vddrf(void) {
  uint8_t rege = (uint8_t)g_pim.vddrf_rege;
  if (rege == 0xFF) return;                       /* 自動のまま */
  if (rege > 15) rege = 15;
  regWrite(REG_REGULATOR_CONTROL, (uint8_t)(0x80 | (rege << 3)));   /* reg_s=1 + rege */
  bool sup3v = (regRead(REG_IO_CONF2) & IO2_sup3V) != 0;
  mlogf("[vddrf] manual rege=%u -> %umV%s\r\n", rege, pim_vddrf_mv(rege, sup3v),
        sup3v ? " (sup3V=1: 3.3V 電源なので 5V は出せない)" : "");
}

void pim_field_on() {
  regSet(REG_OP_CONTROL, OPC_en | OPC_tx_en | OPC_rx_en);
  uint8_t aux = regRead(REG_AUX_DISPLAY);
  mlogf("[field] ON  AUX=0x%02X (tx_on=%d rx_on=%d)\r\n", aux, !!(aux & AUX_tx_on), !!(aux & AUX_rx_on));
}
void pim_field_off() { regClr(REG_OP_CONTROL, OPC_tx_en | OPC_rx_en); mlogf("[field] OFF\r\n"); }

static void pim_rx_config() {
  for (int i = 0; i < 4; i++) {
    uint8_t v = (uint8_t)(g_pim.rxconf >> (8 * i));
    if (v != 0xFF) regWrite(REG_RX_CONF1 + i, v);
  }
  if ((uint8_t)g_pim.txpwr != 0xFF) regWrite(REG_RFO_AM_OFF_LEVEL, (uint8_t)g_pim.txpwr);
  pim_apply_vddrf();          /* ★SET_DEFAULT で消えるので磁界を入れ直すたびに書く */
  /* ★RX_CONF4 のゲイン設定は RESET_RX_GAIN を送らないと AGC/Squelch/RSSI に読み込まれない
   *   （DS11793 p.63 注 2）。同時に «進行中の Squelch を止めて減衰を clear» するので、
   *   受信 FE を入れ直すたびにゲインが既知の状態から始まる。 */
  cmdExec(CMD_RESET_RX_GAIN);
  /* ch_sel(bit7): 1 = PM チャネル。«AM» ではない（DS11793 p.61 Table 36）。 */
  mlogf("[rx] PIM RX config: RX_CONF1=0x%02X (%s ch, 424kHz SC) CONF2=0x%02X (AGC %s, squelch %s)\r\n",
        regRead(REG_RX_CONF1), (regRead(REG_RX_CONF1) & 0x80) ? "PM" : "AM",
        regRead(REG_RX_CONF2), (regRead(REG_RX_CONF2) & 0x10) ? "on" : "off",
        (regRead(REG_RX_CONF2) & 0x02) ? "auto" : "off");
  mlogf("[rx] gain: CONF3=0x%02X (1st stage pm=%u) CONF4=0x%02X (2nd/3rd pm=%u) red=0x%02X\r\n",
        regRead(REG_RX_CONF3), (unsigned)((regRead(REG_RX_CONF3) >> 2) & 7u),
        regRead(REG_RX_CONF4), (unsigned)(regRead(REG_RX_CONF4) & 0x0Fu),
        regRead(REG_GAIN_RED_STATE));
}

/* g_pim.rxconf の 1 バイトだけ差し替える（i=0..3 -> RX_CONF1..4）。0xFF = 書かない。 */
static void pim_rxconf_set_byte(int i, uint8_t v) {
  uint32_t m = 0xFFu << (8 * i);
  g_pim.rxconf = (g_pim.rxconf & ~m) | ((uint32_t)v << (8 * i));
}
static uint8_t pim_rxconf_byte(int i) { return (uint8_t)(g_pim.rxconf >> (8 * i)); }

/* ===========================================================================
 *  transparent モードのピン設定
 * =========================================================================== */
static void tpm_pins_observe() {
  pinMode(PA5, OUTPUT); digitalWrite(PA5, HIGH);   /* rx_on = high */
  pinMode(PA7, OUTPUT); digitalWrite(PA7, LOW);    /* mod off      */
}
static void tpm_pins_restore_spi() { SPI.end(); SPI.begin(); }

void pim_transparent_enter() {
  pim_rx_config();
  pim_field_on();
  mlogf("[tpm] -> transparent (0xDC). AM=MISO(PA6/D12) が 424kHz サブキャリア\r\n");
  cmdExec(CMD_TRANSPARENT_MODE);
  tpm_pins_observe();
  m_tpm = 1;
}
void pim_transparent_exit() {
  tpm_pins_restore_spi();
  cmdExec(CMD_SET_DEFAULT);
  m_tpm = 0;
  mlogf("[tpm] exited (SET_DEFAULT).\r\n");
}

/* ===========================================================================
 *  リング捕捉: SPI1 master-RX-only + DMA1_Channel2 循環モード
 *    ピン・SPI レジスタの設定手順は multitag の pim_capture_spi_once() と同一。
 *    違いは DMA_CCR_CIRC で止めないことと、1 周ごとの TC 割り込みで周回数を数えること。
 * ==========================================================================*/
static uint8_t           m_listen_hw = 0;
static uint32_t          m_save_cr1 = 0, m_save_cr2 = 0;
extern volatile uint32_t g_ring_wraps;   /* ring_isr.cpp の TC 割り込みが数える周回数 */
static uint8_t           m_pop8[256];

void pim_capture_init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  for (unsigned v = 0; v < 256; v++) {
    uint8_t c = 0;
    for (unsigned b = v; b; b >>= 1) c += (uint8_t)(b & 1u);
    m_pop8[v] = c;
  }
  NVIC_SetPriority(DMA1_Channel2_IRQn, 6);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/* 捕捉開始からの通算 word 数（= DMA が書いた総量）。uint32 で回るが、差だけを使うので問題ない。
 * CNDTR の巻き戻りと TC 割り込みの間には隙間があるので、フラグが立ったまま位置が小さければ
 * 「巻き戻ったが割り込みがまだ」とみなして 1 周足す。 */
static uint32_t ring_total(void) {
  for (;;) {
    uint32_t w    = g_ring_wraps;
    uint32_t n    = DMA1_Channel2->CNDTR & 0xFFFFu;
    uint32_t pend = DMA1->ISR & DMA_ISR_TCIF2;
    if (w != g_ring_wraps) continue;
    if (n == 0u || n > PIM_RING_N) n = PIM_RING_N;
    uint32_t pos = PIM_RING_N - n;
    if (pend && pos < (PIM_RING_N >> 1)) w++;
    return w * PIM_RING_N + pos;
  }
}

static void scan_reset(void);

void pim_listen_start() {
  if (m_listen_hw) return;
  GPIOA->MODER  = (GPIOA->MODER  & ~(3u << 12)) | (2u << 12);   /* PA6 -> AF5 (MISO) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFu << 24)) | (5u << 24);
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->OSPEEDR |= (3u << 6);
  GPIOB->AFR[0]  = (GPIOB->AFR[0] & ~(0xFu << 12)) | (5u << 12); /* PB3 AF5 = SPI1_SCK */
  GPIOB->MODER   = (GPIOB->MODER  & ~(3u << 6))   | (2u << 6);

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  m_save_cr1 = SPI1->CR1; m_save_cr2 = SPI1->CR2;             /* Arduino SPI 設定を退避 */
  SPI1->CR1 = 0;
  SPI1->CR2 = (0xFu << 8) | SPI_CR2_RXDMAEN;                  /* DS=16bit, RX DMA        */

  DMA1_Channel2->CCR = 0;
  DMA1->IFCR = DMA_IFCR_CGIF2;
  DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~(0xFu << 4)) | (0x1u << 4);   /* SPI1_RX */
  DMA1_Channel2->CPAR  = (uint32_t)&SPI1->DR;
  DMA1_Channel2->CMAR  = (uint32_t)g_ring;
  DMA1_Channel2->CNDTR = PIM_RING_N;
  g_ring_wraps = 0;
  DMA1_Channel2->CCR   = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC |
                         DMA_CCR_TCIE | DMA_CCR_PL_0 | DMA_CCR_PL_1 | DMA_CCR_EN;

  uint32_t br = g_pim.spi_br & 7u;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_RXONLY | SPI_CR1_SSM | SPI_CR1_SSI | (br << 3);
  SPI1->CR1 |= SPI_CR1_SPE;

  g_pim.capspi_fs = HAL_RCC_GetPCLK2Freq() / (2u << br);
  scan_reset();
  m_listen_hw = 1;
  g_pim.listen_on = 1;
}

void pim_listen_stop() {
  if (!m_listen_hw) return;
  SPI1->CR1 &= ~SPI_CR1_SPE;
  DMA1_Channel2->CCR = 0;
  DMA1->IFCR = DMA_IFCR_CGIF2;
  while (SPI1->SR & SPI_SR_RXNE) { (void)SPI1->DR; }
  SPI1->CR2 = m_save_cr2;
  SPI1->CR1 = m_save_cr1;
  GPIOA->MODER &= ~(3u << 12);
  GPIOB->MODER &= ~(3u << 6);
  m_listen_hw = 0;
}

/* リングの [start, start+n) word を g_seg に線形に並べ直す（巻き戻りを跨いでもよい） */
static void ring_copy(uint32_t start, uint32_t n) {
  uint32_t i0 = start & PIM_RING_MASK;
  uint32_t first = PIM_RING_N - i0;
  if (first > n) first = n;
  memcpy(g_seg, &g_ring[i0], first * sizeof(uint16_t));
  if (n > first) memcpy(g_seg + first, g_ring, (n - first) * sizeof(uint16_t));
}

/* ===========================================================================
 *  2 段目: セグメント復号（firmware-multitag の pim_decode と同一アルゴリズム）
 * ==========================================================================*/
#define PIM_SPS_MIN 8       /* pim.py の自己テストは sps=16 で通る。10 Msps で sps≈23.6 */
#define PIM_SPS_MAX 160
#define PIM_MAXSOF  40
static float m_costab[PIM_SPS_MAX], m_sintab[PIM_SPS_MAX];
/* ★2026-09-28: 包絡線は «固定小数点» で回す（送信周期 50 ms 対応）。
 *   float + サンプルごとの範囲チェックだと 1 サンプル ~200 クロックかかり、
 *   1 セグメント（平均 5400 sample）の復号が 11-15 ms になって 50 ms 周期に
 *   追いつかなかった。係数を 2^12 倍の整数にし、I/Q を int32、電力を int64 で持つ。
 *   量子化誤差は係数の 1/8192 で、閾値判定にはまったく効かない。
 *   値の «意味»（振幅 = 2*sqrt(I^2+Q^2)/sps、閾値 = 0.5*最大）は float 版と同一。 */
#define PIM_TAB_SHIFT 12
#define PIM_TAB_ONE   (1 << PIM_TAB_SHIFT)
static int32_t m_icos[PIM_SPS_MAX], m_isin[PIM_SPS_MAX];
static int   m_tab_sps = 0;
static float m_thr;
static int64_t m_thr_pow_i;          /* 電力の閾値（固定小数点の 2 乗単位）*/

/* ★計装（2026-09-15）: 「SOF は 100% 検出できるのに ID だけ落ちる」理由を直接見る。
 *   `diag <n>` で次の n セグメント分だけ、閾値・SOF の振幅・ID 16 チップそれぞれの
 *   振幅を出す。チップが閾値のどちら側に «どれだけの余裕で» 座っているかが分かる。
 *   常時出すと UART が律速になるので、ワンショットのカウンタで回数を絞る。 */
static uint32_t m_diag = 0;

/* ★計装: ID のサンプル位置に足す符号付きオフセット [sample]。`idoff <n>` で設定。
 *   body0 = sof_start + (SOF長+gap)*sps + m_idoff。診断用（0 が現行動作）。 */
static int m_idoff = 0;

/* sps_milli=0 は「実捕捉レートから算出」。multitag は算出値を sps_milli に書き戻して
 * 固定化していたが、この版は `rate` で捕捉レートを変えるので書き戻さない。 */
/* ★2026-09-28: 送信周期 50 ms 対応の計装。
 *   m_seg_t0_us : いま復号しているセグメントの «先頭サンプル» の時刻 [us]（micros() 基準）。
 *                 復号は数 ms〜数十 ms 遅れて走るので、復号時の micros() から «リング上で
 *                 どれだけ前のサンプルか» を引いて求める（サンプル精度）。
 *   m_us_per_samp: 1 サンプルの長さ [us]（捕捉レートの逆数）。 */
static uint32_t m_seg_t0_us = 0;
static uint32_t m_prof[4];            /* 計装: 復号の内訳 [us] 0=scan1 1=scan2 2=read_id 3=nsof 合計 */
static float    m_us_per_samp = 0.2f;

static uint32_t sps_milli_eff(void) {
  if (g_pim.sps_milli) return g_pim.sps_milli;
  if (!g_pim.capspi_fs) return 23600u;
  return (uint32_t)((double)g_pim.capspi_fs * 1000.0 / PIM_FSC_HZ + 0.5);
}

static inline int pim_bit01(int s) {
  return (g_seg[s >> 4] >> (15 - (s & 15))) & 1;                       /* MSB first */
}
static inline int pim_bit_pm(int s) { return pim_bit01(s) ? 1 : -1; }  /* ±1 */
static float pim_iq_mag(int c, int sps, int N) {
  int s0 = c - sps / 2; int32_t I = 0, Q = 0;
  int idx = ((s0 % sps) + sps) % sps;        /* 剰余は 1 回だけ。以降は折り返しで進める */
  int m0 = 0, m1 = sps;                      /* 範囲外のサンプルは 0 扱い（端だけ削る） */
  if (s0 < 0)        { m0 = -s0; idx = (idx + m0) % sps; }
  if (s0 + sps > N)  { m1 = N - s0; }
  for (int m = m0; m < m1; m++) {
    if (pim_bit01(s0 + m)) { I += m_icos[idx]; Q += m_isin[idx]; }
    else                   { I -= m_icos[idx]; Q -= m_isin[idx]; }
    if (++idx == sps) idx = 0;
  }
  float fi = (float)I, fq = (float)Q;
  return 2.0f * sqrtf(fi * fi + fq * fq) / ((float)sps * (float)PIM_TAB_ONE);
}
/* ★2026-09-28: ID 読み用の振幅キャッシュ。pim_read_id は ±半チップ × 16 チップの
 *   位置で pim_iq_mag を ~176 回呼び、同じ窓を何度も足し直していた（1 SOF ~1 ms）。
 *   探索範囲の振幅を running sum で 1 回だけ求めて表から引く。値は pim_iq_mag と同一。 */
#define PIM_AMPC_N 512
static float m_ampc[PIM_AMPC_N];
static int   m_ampc_lo = 0, m_ampc_n = 0;
static void pim_amp_fill(int lo, int hi, int sps, int N) {
  int n = hi - lo + 1;
  m_ampc_lo = lo; m_ampc_n = 0;
  if (n <= 0 || n > PIM_AMPC_N) return;             /* 入らなければ従来どおり都度計算 */
  const int half = sps / 2;
  int32_t I = 0, Q = 0;
  int s0 = lo - half;
  int kwl = ((s0 % sps) + sps) % sps;
  for (int m = 0, idx = kwl; m < sps; m++) {
    int ss = s0 + m;
    if (ss >= 0 && ss < N) { int v = pim_bit_pm(ss); I += v * m_icos[idx]; Q += v * m_isin[idx]; }
    if (++idx == sps) idx = 0;
  }
  const float k = 2.0f / ((float)sps * (float)PIM_TAB_ONE);
  for (int j = 0; j < n; j++) {
    float fi = (float)I, fq = (float)Q;
    m_ampc[j] = k * sqrtf(fi * fi + fq * fq);
    int wl = lo + j - half;                          /* 窓を 1 サンプル進める */
    int vout = (wl >= 0 && wl < N) ? pim_bit_pm(wl) : 0;
    int vin  = (wl + sps >= 0 && wl + sps < N) ? pim_bit_pm(wl + sps) : 0;
    int dv = vin - vout;
    if (dv) { I += dv * m_icos[kwl]; Q += dv * m_isin[kwl]; }
    if (++kwl == sps) kwl = 0;
  }
  m_ampc_n = n;
}
static inline float pim_amp(int c, int sps, int N) {
  unsigned j = (unsigned)(c - m_ampc_lo);
  return (j < (unsigned)m_ampc_n) ? m_ampc[j] : pim_iq_mag(c, sps, N);
}

static inline int pim_sof_tu_of(int kind) {
  return (kind == PIM_KIND_PREAMBLE) ? PIM_SOF_PRE_TU : PIM_SOF_POST_TU;
}
/* ★2026-09-15: chip 位置は «小数 sps» で出すこと。整数 sps(24 vs 真値 23.7) だと
 *   body0 で +1.5〜2.7 sample、chip15 では +6.2〜7.4 sample ＝ 1 チップの 26〜31% ずれ、
 *   窓が隣のチップにまたがって Manchester 違反を量産する。間隔計算では既に小数を
 *   使っていたが（spsf）、ID 読みだけ整数のまま取り残されていた。
 *   実測: preamble の ID が毎回失敗し postamble だけ通る、という非対称の正体。 */
/* ★2026-09-15: タイミング再生つき ID 読み。
 *
 *   旧実装は 16 チップの位置を «sof_start + 公称長 x sps» だけで決めていた。
 *   実測（`idoff` スイープ）では、正しく読める窓は **±6 sample / 1chip=23.6 sample
 *   = ±25%** しかなく、しかも最適位置は経路で動いた（AD3 を再プログラムすると
 *   分周器の位相が変わり -8 sample ずれる → 窓の外 → 復号率 0%）。SOF は 4-8 TU
 *   あるので同じずれを吸収でき、「SOF は 100% 見えるのに ID だけ全滅」になっていた。
 *
 *   ここでは公称位置を «出発点» にして ±半チップをスキャンし、
 *     (a) Manchester として成立する
 *     (b) 全チップが閾値から最も «遠い»（= 判定が最も決定的）
 *   を満たすグリッドを選ぶ。受信機が普通に持っているビット同期の代わりで、
 *   ST25R3916 ならシリコンがやってくれる部分にあたる。 */
/* ★2026-09-25: チップ座標を «小数 sps (spsf)» で計算する。
 *   上のコメントどおり «小数でやること» と分かっていたのに、実装は int sps のままで
 *   残っていた（間隔計算だけが spsf を使っていた）。sps=12 と真値 11.85 の差 0.15
 *   sample/TU は、ID 16 チップの間に 2.4 sample = 1 チップの 20% まで «溜まる»
 *   線形ドリフトなので、一様オフセットの探索では直せない（先頭を合わせると末尾が
 *   外れる）。40 Msps では sps=94 と 94.4 で同じ問題が出る。 */
static int pim_read_id(int sof_start, int kind, int sps, float spsf, int N,
                       int *id, int *body_out) {
  int base = sof_start + (int)lroundf((float)(pim_sof_tu_of(kind) + PIM_GAP_TU) * spsf)
             + m_idoff;
  int half = (int)(PIM_ID_SEARCH_CHIPS * spsf + 0.5f);          /* 既定 ±0.5 chip */
  if (half > 64) half = 64;                                      /* scores[] の大きさ */
  /* チップ k の «中央» のサンプル位置（base 相対）。小数 sps をそのまま使う。 */
  #define PIM_CHIP_CENTER(k) ((int)lroundf(((float)(k) + 0.5f) * spsf))
  pim_amp_fill(base - half + PIM_CHIP_CENTER(0), base + half + PIM_CHIP_CENTER(2 * PIM_ID_BITS - 1),
               sps, N);

  /* --- 1 段目: «タイミングだけ» を決める ------------------------------------
   *   スコア = 全チップの |振幅 - 閾値| の和。正しい位置では各チップが完全に
   *   ON か OFF に振り切れて最大、ずれた位置では窓が隣のチップにまたがって
   *   閾値付近に潰れるので小さい。Manchester が «成立するか» は使わない。
   *   ここで成立性を混ぜると、ずれた位置でたまたま成立する別の ID を掴む
   *   （2026-09-15 実測: 誤り 0-2 -> 25-51 に増えた）。 */
  /* ★2026-09-28: 全オフセットを評価して «台地の中央» を採る。
   *   スコアは正しい位置の周りで平らな台地になる（チップ中央付近ならどこでも
   *   |ma - mb| が同じ）。旧実装は粗 -> 微の 2 段で «最初の最大» を採っていたので、
   *   台地のどこを採るかが丸め誤差次第だった。body はそのまま間隔測定に使うので、
   *   台地の端に寄ると preamble/postamble で数 sample ずれて data が ±1 化ける
   *   （固定小数点化で同点が «常に左端» になり、実機の誤りが 3 倍に増えて露呈した）。
   *   振幅はキャッシュから引くので全探索でも軽い（±6 で 13 通り x 16 チップ）。 */
  float scores[2 * 64 + 1];
  int   best_off = 0;
  float best_score = -1.0f;
  for (int off = -half; off <= half; off++) {
    float score = 0.0f;
    int   bad = 0;
    /* Manchester は «1 ビット = 必ず ON 1 個 + OFF 1 個» なので、ビット内 2 チップの
       振幅差が最大になる位置が正しいタイミング。
         正しい位置    : |満 - 空| で各ビット最大
         半チップずれ  : 両方が中間値になり差 ~ 0
         1 チップずれ  : ペアがビットをまたぎ (0,0)/(1,1) になって差 ~ 0
       単なる |振幅-閾値| の総和では «1 チップずれ» を区別できない（実測で確認）。 */
    for (int i = 0; i < PIM_ID_BITS; i++) {
      int ca = base + off + PIM_CHIP_CENTER(2 * i);
      int cb = base + off + PIM_CHIP_CENTER(2 * i + 1);
      if (ca < 0 || cb >= N) { bad = 1; break; }
      float ma = pim_amp(ca, sps, N);
      float mb = pim_amp(cb, sps, N);
      score += (ma > mb) ? (ma - mb) : (mb - ma);
    }
    scores[off + half] = bad ? -1.0f : score;
    if (bad) continue;
    if (score > best_score) { best_score = score; best_off = off; }
  }
  if (best_score < 0.0f) return 0;
  /* 最大と «ほぼ同点»（相対 1e-3 以内）が続く範囲 = 台地。その中央を採る。 */
  {
    const float tol = best_score * 1.0e-3f;
    int l = best_off, r = best_off;
    while (l - 1 >= -half && scores[l - 1 + half] >= best_score - tol) l--;
    while (r + 1 <=  half && scores[r + 1 + half] >= best_score - tol) r++;
    best_off = (l + r) / 2;                 /* 偶数幅なら左寄りの中央（系統差は 0.5 sample 未満） */
  }

  /* --- 2 段目: 決めた位置«だけ»で復号する（成立しなければ素直に失敗）-------
   *
   *  ★2026-09-25: 判定を «絶対しきい値» から «ビット内 2 チップの比較» に変えた。
   *
   *    m_thr = 0.5 x セグメント最大振幅 で、最大は必ず SOF（4〜8 TU の連続バースト）
   *    から来る。SOF は 1 TU の LPF を完全に満たすので振幅が飽和するが、ID の 1 TU
   *    チップは立ち上がり/立ち下がりの分だけ必ず目減りする。結合が落ちると
   *    «SOF は 199% なのにチップは 46%» になり、SOF は毎回見えるのに ID だけ全滅する
   *    （メアンダコイル服 + タグ 2 枚で実測: ID 成功 1/20 -> 17/20）。
   *
   *    Manchester の 1 ビットは «ON 1 chip + OFF 1 chip» で必ず出来ているので、
   *    2 チップの «どちらが大きいか» だけでビットが決まる。絶対振幅に依らないので
   *    結合の変動・ISI による目減りにそのまま強い。1 段目のタイミング再生が既に
   *    同じ |ma - mb| を指標にしているので、指標と判定が一致して筋も通る。
   *
   *    コントラスト下限（PIM_ID_CHIP_CONTRAST x m_thr）だけは残す。両チップとも
   *    無信号のときに «大きい方» を拾ってコイン投げの ID を作らないため。 */
  int u = 0;
  const float mincon = PIM_ID_CHIP_CONTRAST * m_thr;
  for (int i = 0; i < PIM_ID_BITS; i++) {
    int ca = base + best_off + PIM_CHIP_CENTER(2 * i);
    int cb = base + best_off + PIM_CHIP_CENTER(2 * i + 1);
    if (ca < 0 || cb >= N) return 0;
    float d = pim_amp(cb, sps, N) - pim_amp(ca, sps, N);         /* bit 1 -> [0,1] */
    if (d < 0.0f ? -d < mincon : d < mincon) return 0;
    u = (u << 1) | (d > 0.0f ? 1 : 0);
  }
  *id = u;
  /* ★間隔測定にも «再生した» 位置を使う。sof_start は SOF 長で検出バイアスが
     変わる（実測: run 長 PRE 4.00 TU / POST 8.45 TU）ため、preamble と postamble で
     バイアスが非対称になり data が -1 ずれていた（実測: data=100 が 99 に）。
     body は ID 先頭という «両パルス共通の特徴» なので、バイアスが相殺される。 */
  if (body_out) *body_out = base + best_off;
  return 1;
}
#undef PIM_CHIP_CENTER

/* ★計装: 1 セグメント分の「閾値 vs 実振幅」を UART に出す。
 *
 *   thr      = 0.5 x (セグメント内の最大振幅)。最大は普通 SOF から来る。
 *   sof[i]   = その SOF の «連続 ON» 区間の長さ [TU x100] と、区間内の代表振幅
 *   chip[k]  = ID 16 チップそれぞれの振幅。thr との比 (x100) を併記する。
 *              100 を超えていれば 1 と判定される。100 付近に密集していれば
 *              「閾値の上に辛うじて乗っているだけ」＝ノイズで簡単に反転する状態。
 *   浮動小数の %f は newlib-nano で出ないので、全部 x1000 / x100 の整数で出す。 */
static void pim_diag_dump(const int *sof_start, const int *sof_kind, int nsof,
                          int sps, float spsf, int N, float gmax_pow) {
  float gmax_amp = 2.0f * sqrtf(gmax_pow) / (float)sps;
  logf("[diag] sps=%d N=%d  gmax_amp=%ld thr=%ld (x1000)  nsof=%d\r\n",
       sps, N, (long)(gmax_amp * 1000.0f), (long)(m_thr * 1000.0f), nsof);

  for (int i = 0; i < nsof && i < 4; i++) {
    /* SOF の連続 ON 長を数え直す（sof_scan は長さを返さないので実測する） */
    int run = 0; float sof_peak = 0.0f;
    for (int c = sof_start[i]; c < N; c++) {
      float m = pim_iq_mag(c, sps, N);
      if (m <= m_thr) break;
      if (m > sof_peak) sof_peak = m;
      run++;
    }
    logf("[diag] sof%d start=%d run=%ld/100TU kind=%s peak=%ld (%ld%% of thr)\r\n",
         i, sof_start[i], (long)(100.0f * (float)run / (float)sps),
         sof_kind[i] == PIM_KIND_PREAMBLE ? "PRE " : "POST",
         (long)(sof_peak * 1000.0f), (long)(100.0f * sof_peak / m_thr));

    /* ID 16 チップの振幅を、pim_read_id と «同じ座標計算» で読む */
    int body0 = sof_start[i]
                + (int)lroundf((float)(pim_sof_tu_of(sof_kind[i]) + PIM_GAP_TU) * spsf)
                + m_idoff;
    logf("[diag]   chip%%thr:");
    for (int k = 0; k < PIM_ID_TU; k++) {
      int c = body0 + (int)lroundf(((float)k + 0.5f) * spsf);
      long pct = (c < N) ? (long)(100.0f * pim_iq_mag(c, sps, N) / m_thr) : -1;
      logf(" %ld", pct);
    }
    logf("\r\n[diag]   chips   :");
    int viol = -1;
    for (int k = 0; k < PIM_ID_TU; k++) {
      int c = body0 + (int)lroundf(((float)k + 0.5f) * spsf);
      int bit = (c < N && pim_iq_mag(c, sps, N) > m_thr) ? 1 : 0;
      logf(" %d", bit);
      if ((k & 1) == 1 && viol < 0) {
        int ca = body0 + (int)lroundf(((float)(k - 1) + 0.5f) * spsf);
        int a = (ca < N && pim_iq_mag(ca, sps, N) > m_thr) ? 1 : 0;
        if (a == bit) viol = k / 2;      /* Manchester 違反が出た «ビット番号» */
      }
    }
    /* ★この違反は «絶対しきい値で 0/1 にしたら» の話。実際の判定はビット内 2 チップの
       比較なので（pim_read_id 2 段目）、ここが >=0 でも復号は成功しうる。上の chip%%thr
       で «ビット内の 2 つが十分に差が付いているか» を見るのが正しい読み方。 */
    logf("   thr_only_violation_at_bit=%d\r\n", viol);
  }
}

/* 閾値を超えた連続区間（run）を SOF として記録する。run 長 [TU] で種別を決める。 */
static inline void pim_sof_close(int run, int run_start, int sps,
                                 int *sof_start, int *sof_kind, int *nsof, int *total) {
  float run_tu = (float)run / (float)sps;
  if (run_tu < (float)PIM_SOF_MIN_TU) return;
  (*total)++;
  if (*nsof < PIM_MAXSOF) {
    sof_start[*nsof] = run_start;
    sof_kind[*nsof]  = (run_tu < PIM_SOF_CLASS_TU) ? PIM_KIND_PREAMBLE : PIM_KIND_POSTAMBLE;
    (*nsof)++;
  }
}

/* running-sum IQ 包絡線（窓 = 1 TU = sps サンプル、中心 c）を 1 サンプルずつ進める。
 *   find_gmax=1 : 最大電力だけを求める（gmax_out, float 版と同じ単位で返す）
 *   find_gmax=0 : 電力 > m_thr_pow_i の連続区間を SOF として拾う
 * 入るサンプル(wl+sps)と出るサンプル(wl)は sps 離れで係数が同じなので、差 dv だけで更新。
 * 中央区間（両端とも範囲内）は範囲チェック無しの速いループで回す。 */
static void pim_sof_scan(int sps, int N, int find_gmax,
                         float *gmax_out, int *sof_start, int *sof_kind,
                         int *nsof_out, int *total_out) {
  const int half = sps / 2;
  int32_t I = 0, Q = 0;
  for (int s = -half; s < -half + sps; s++)
    if (s >= 0 && s < N) {
      int k = ((s % sps) + sps) % sps, v = pim_bit_pm(s);
      I += v * m_icos[k]; Q += v * m_isin[k];
    }
  int kwl = (((-half) % sps) + sps) % sps;
  int64_t gmax = 0;
  const int64_t thr = m_thr_pow_i;
  int nsof = 0, total = 0, run = 0, run_start = 0;
  /* 中央区間 = wl >= 0 かつ wl + sps < N  <=>  half <= c < N - sps + half */
  const int c_mid0 = half, c_mid1 = N - sps + half;
  /* ★無音の読み飛ばし（結果は読み飛ばさない場合と «完全に同一»）。
   *   セグメントの大半は無音（先頭/末尾のブロック、preamble と postamble の間の
   *   data×TU）で、Transparent の出力ビットは一定になる。窓の中が全部同じ値 v なら
   *   I = v*Σcos, Q = v*Σsin で電力は定数 C0（≒0）。そこで «16 サンプル + 次の窓» が
   *   全部同じ値のワードなら、16 サンプルを 1 回で進めて I/Q を厳密値に置き直す。
   *   実測（2026-09-28）: 1 セグメント 5400 sample のうち信号は ~550 sample。 */
  int32_t SC = 0, SS = 0;
  for (int k = 0; k < sps; k++) { SC += m_icos[k]; SS += m_isin[k]; }
  const int64_t C0 = (int64_t)SC * SC + (int64_t)SS * SS;

  for (int c = 0; c < N; c++) {
    if (((c & 15) == 0) && c >= c_mid0 && c + 16 <= c_mid1) {
      /* 見るサンプル = c..c+15 の窓 + c+16 の窓 = [c-half, c+16-half+sps) */
      int w0 = (c - half) >> 4, w1 = (c + 16 - half + sps - 1) >> 4;
      uint16_t w = g_seg[w0];
      bool flat = (w == 0x0000u || w == 0xFFFFu);
      for (int k = w0 + 1; flat && k <= w1; k++) flat = (g_seg[k] == w);
      if (flat) {
        if (find_gmax) { if (C0 > gmax) gmax = C0; }
        else if (C0 > thr) { if (run == 0) run_start = c; run += 16; }
        else if (run > 0) { pim_sof_close(run, run_start, sps, sof_start, sof_kind, &nsof, &total); run = 0; }
        int v = w ? 1 : -1;
        I = v * SC; Q = v * SS;                     /* c+16 の窓も全部 v なので厳密 */
        kwl = (kwl + 16) % sps;
        c += 15;                                    /* for の c++ と合わせて +16 */
        continue;
      }
    }
    int64_t pw = (int64_t)I * I + (int64_t)Q * Q;
    if (find_gmax) { if (pw > gmax) gmax = pw; }
    else if (pw > thr) { if (run == 0) run_start = c; run++; }
    else if (run > 0) { pim_sof_close(run, run_start, sps, sof_start, sof_kind, &nsof, &total); run = 0; }

    int wl = c - half, dv;
    if (c >= c_mid0 && c < c_mid1) {
      dv = pim_bit01(wl + sps) - pim_bit01(wl);          /* (±1 の差)/2 : -1, 0, +1 */
      dv <<= 1;
    } else {
      int vout = (wl >= 0 && wl < N) ? pim_bit_pm(wl) : 0;
      int vin  = (wl + sps >= 0 && wl + sps < N) ? pim_bit_pm(wl + sps) : 0;
      dv = vin - vout;
    }
    if (dv) { I += dv * m_icos[kwl]; Q += dv * m_isin[kwl]; }
    if (++kwl == sps) kwl = 0;
  }
  if (!find_gmax && run > 0) pim_sof_close(run, run_start, sps, sof_start, sof_kind, &nsof, &total);
  if (gmax_out)  *gmax_out  = (float)gmax / ((float)PIM_TAB_ONE * (float)PIM_TAB_ONE);
  if (find_gmax) m_thr_pow_i = gmax / 4;                /* 閾値 = 0.5 x 最大振幅 (電力で 1/4) */
  if (nsof_out)  *nsof_out  = nsof;
  if (total_out) *total_out = total;
}

/* g_seg の先頭 N sample を復号して g_tags / g_pim に書く */
static void pim_decode_seg(int N) {
  g_pim.valid = 0; g_pim.sof_count = 0; g_pim.ntags = 0;
  uint32_t spsm = sps_milli_eff();
  int sps = (int)((spsm + 500) / 1000);
  if (sps < PIM_SPS_MIN) sps = PIM_SPS_MIN;
  if (sps > PIM_SPS_MAX - 1) sps = PIM_SPS_MAX - 1;
  int mult = (int)g_pim.data_tu_mult; if (mult < 1) mult = 1;

  if (m_tab_sps != sps) {
    for (int m = 0; m < sps; m++) {
      float a = 2.0f * 3.14159265358979f * (float)m / (float)sps;
      m_costab[m] = cosf(a); m_sintab[m] = sinf(a);
      m_icos[m] = (int32_t)lroundf(m_costab[m] * (float)PIM_TAB_ONE);
      m_isin[m] = (int32_t)lroundf(m_sintab[m] * (float)PIM_TAB_ONE);
    }
    m_tab_sps = sps;
  }

  float gmax_pow = 0.0f;
  uint32_t tp0 = micros();
  pim_sof_scan(sps, N, 1, &gmax_pow, NULL, NULL, NULL, NULL);
  m_prof[0] += micros() - tp0;
  if (gmax_pow <= 0.0f) { g_pim.done++; return; }
  m_thr     = sqrtf(gmax_pow) / (float)sps;      /* m_thr_pow_i は pim_sof_scan が設定済み */

  int sof_start[PIM_MAXSOF], sof_kind[PIM_MAXSOF], nsof = 0, total_sof = 0;
  tp0 = micros();
  pim_sof_scan(sps, N, 0, NULL, sof_start, sof_kind, &nsof, &total_sof);
  m_prof[1] += micros() - tp0;
  m_prof[3] += (uint32_t)nsof;
  g_pim.sof_count = (uint32_t)total_sof;
  if (nsof < 2) { g_pim.done++; return; }

  float spsf = (float)spsm / 1000.0f;   /* 小数 sps（整数だと間隔も ID 位置もずれる） */

  int uid[PIM_MAXSOF], okp[PIM_MAXSOF], body[PIM_MAXSOF];
  tp0 = micros();
  for (int i = 0; i < nsof; i++)
    okp[i] = pim_read_id(sof_start[i], sof_kind[i], sps, spsf, N, &uid[i], &body[i]);
  m_prof[2] += micros() - tp0;

  if (m_diag) { m_diag--; pim_diag_dump(sof_start, sof_kind, nsof, sps, spsf, N, gmax_pow); }
  uint8_t used[PIM_MAXSOF];
  for (int i = 0; i < nsof; i++) used[i] = 0;

  uint32_t ntags = 0;
  for (int i = 0; i < nsof && ntags < PIM_MAXTAGS; i++) {
    if (!okp[i] || used[i] || sof_kind[i] != PIM_KIND_PREAMBLE) continue;
    int j = -1;
    for (int k = i + 1; k < nsof; k++) {
      if (!okp[k] || used[k]) continue;
      if (sof_kind[k] == PIM_KIND_POSTAMBLE && uid[k] == uid[i]) { j = k; break; }
    }
    if (j < 0) continue;

    /* body(ID 先頭) 同士の差から出す。SOF 長の差 (POST-PRE) を引けばパルス頭の差になる。 */
    float interval_tu = ((float)(body[j] - body[i])) / spsf
                        - (float)(PIM_SOF_POST_TU - PIM_SOF_PRE_TU) - (float)PIM_PRE_PULSE_TU;
    int data = (int)(interval_tu / (float)mult + 0.5f);
    if (data < 1 || data > 255) continue;

    used[i] = used[j] = 1;
    g_tags[ntags].tag_id      = (uint32_t)uid[i];
    g_tags[ntags].data        = (uint32_t)data;
    g_tags[ntags].interval_tu = (int32_t)(interval_tu + 0.5f);
    /* preamble の «パルス先頭» = ID 先頭(body) から SOF+gap を戻した位置 */
    float pre0 = (float)body[i] - (float)(PIM_SOF_PRE_TU + PIM_GAP_TU) * spsf;
    g_tags[ntags].t_us = m_seg_t0_us + (uint32_t)(int32_t)lroundf(pre0 * m_us_per_samp);
    ntags++;
  }
  g_pim.ntags = ntags;
  if (ntags > 0) {
    g_pim.tag_id      = g_tags[0].tag_id;
    g_pim.data        = g_tags[0].data;
    g_pim.interval_tu = g_tags[0].interval_tu;
    g_pim.valid       = 1u;
  }
  g_pim.done++;
}

/* host/pim_multitag.py の RX_RE / TAG_RE がそのまま読める書式（multitag と同一） */
static void pim_print_result() {
  /* t= はリーダの micros() [us]。host はこれを発射時刻に写してラウンドを決める
     （送信周期より遅れて届いても取り違えない）。既存の正規表現は後ろの追加欄を無視する。 */
  logf("[rx #%lu] sof=%lu tags=%lu t=%lu",
       (unsigned long)g_pim.done, (unsigned long)g_pim.sof_count, (unsigned long)g_pim.ntags,
       (unsigned long)m_seg_t0_us);
  if (g_pim.ntags == 0) { logf("   (no complete packet)\r\n"); return; }
  logf("\r\n");
  for (uint32_t i = 0; i < g_pim.ntags; i++)
    logf("    tag_id=%3lu  data=%3lu  interval=%ld TU  t=%lu\r\n",
         (unsigned long)g_tags[i].tag_id, (unsigned long)g_tags[i].data,
         (long)g_tags[i].interval_tu, (unsigned long)g_tags[i].t_us);
}

/* ★2026-09-28 計装: 1 セグメントあたりの復号・表示の時間と、走査の遅れ。
 *   50 ms 周期で追いつくには «(復号 + 表示) × セグメント数/s < 1 s» が要る。
 *   `stat` で表示してリセットする。 */
static struct {
  uint32_t dec_n, dec_sum, dec_max;     /* 復号 [us]                               */
  uint32_t prt_n, prt_sum, prt_max;     /* 表示（UART へ書き終わるまで）[us]      */
  uint32_t seg_sum, seg_max;            /* セグメント長 [sample]                   */
  uint32_t lag_max;                     /* 走査位置の遅れの最大 [word]             */
  uint32_t t0_ms;                       /* 集計開始 [ms]                           */
} m_st;

/* リングの [start, start+n) を切り出して復号。上書きされていたら捨てる。 */
static bool seg_decode(uint32_t start, uint32_t n) {
  if (n == 0) return false;
  if (n > PIM_SEG_N) n = PIM_SEG_N;
  if (ring_total() - start > PIM_RING_N) { g_pim.overruns++; return false; }
  ring_copy(start, n);
  uint32_t now_us = micros(), tot = ring_total();
  if (tot - start > PIM_RING_N) { g_pim.overruns++; return false; }   /* コピー中に上書き */
  /* セグメント先頭の時刻 = 今 - (今の書き込み位置 - 先頭) サンプルぶん */
  uint32_t fs = g_pim.capspi_fs ? g_pim.capspi_fs : 5000000u;
  m_us_per_samp = 1.0e6f / (float)fs;
  m_seg_t0_us = now_us - (uint32_t)((uint64_t)(tot - start) * 16u * 1000000u / fs);
  uint32_t t0 = micros();
  pim_decode_seg((int)(n * 16u));
  uint32_t dt = micros() - t0;
  g_pim.decode_us = dt;
  g_pim.segments++;
  m_st.dec_n++; m_st.dec_sum += dt; if (dt > m_st.dec_max) m_st.dec_max = dt;
  if (n * 16u > m_st.seg_max) m_st.seg_max = n * 16u;
  m_st.seg_sum += n * 16u;
  return true;
}

/* ===========================================================================
 *  1 段目: 活動判定 + セグメントの開閉
 *    ブロック（64 word = 1024 sample）ごとに隣接サンプル間のビット遷移を数える。
 *    サブキャリアが出ていれば 1 周期あたり 2 回遷移し、出ていなければ Transparent の
 *    出力はほぼ一定（実測: タグ静止時 トグル 0, PLAN.md §3）なので、遷移数だけで判定できる。
 *
 *    1 サイクル（preamble → 間隔 → postamble）の途中には data×mult TU の無音がある。
 *    これより長い無音が続くまでセグメントを閉じないので、1 サイクルが丸ごと入る。
 * ==========================================================================*/
static uint32_t m_scan_pos = 0;       /* 次に走査する word（通算）       */
static uint16_t m_prev_w = 0xFFFF;    /* 直前 word（ブロック境界の遷移用） */
static uint8_t  m_seg_open = 0;
static uint32_t m_seg_start = 0;      /* 通算 word                       */
static uint32_t m_seg_last_act = 0;   /* 最後の活動ブロックの終端（通算） */
static uint32_t m_seg_idle = 0;       /* 連続無音ブロック数               */
static uint32_t m_hang_blk = 8;       /* 何ブロック無音ならサイクル終了か */
/* ★受信 FE の «詰まり» 検出。SOF は見えているのに 1 タグも取れないセグメントが
   連続したら、受信チェーンが落ちたと判断して即 resync を要求する（下の
   pim_resync_task が実行する。ここで直接呼ぶと走査中の状態を壊す）。 */
static uint32_t m_stuck = 0;          /* 連続で «SOF>0 なのに ntags=0» だった回数 */
static uint8_t  m_resync_req = 0;     /* 1 = 次の pim_resync_task で入れ直す      */
static uint32_t m_last_t = 0, m_max_t = 0;
static uint32_t m_last_act_ms = 0;    /* 最後に活動ブロックを見た時刻 [ms]（無音検出用） */

static void scan_reset(void) {
  m_scan_pos = 0; m_prev_w = 0xFFFF;
  m_seg_open = 0; m_seg_idle = 0;
}

static uint32_t block_transitions(uint32_t pos) {
  uint32_t cnt = 0; uint16_t p = m_prev_w;
  for (uint32_t k = 0; k < PIM_BLK_W; k++) {
    uint16_t w = g_ring[(pos + k) & PIM_RING_MASK];
    uint16_t d = (uint16_t)((w ^ (w >> 1)) & 0x7FFFu);   /* word 内 15 箇所の遷移 */
    cnt += m_pop8[d & 0xFFu] + m_pop8[d >> 8];
    cnt += (uint32_t)((p ^ (w >> 15)) & 1u);              /* 前 word 末尾 → この word 先頭 */
    p = w;
  }
  m_prev_w = p;
  return cnt;
}

/* 1 サイクル内で最長の無音（255×mult TU）より長い無音をサイクル終了とみなす */
static uint32_t hang_blocks(void) {
  int mult = (int)g_pim.data_tu_mult; if (mult < 1) mult = 1;
  float spsf = (float)sps_milli_eff() / 1000.0f;
  float samp = (255.0f * (float)mult + 16.0f) * spsf;
  uint32_t h = (uint32_t)(samp / (float)(PIM_BLK_W * 16u)) + 1u;
  return (h < 4u) ? 4u : h;
}

static void seg_feed(uint32_t blk_pos, bool active) {
  uint32_t blk_end = blk_pos + PIM_BLK_W;
  if (!m_seg_open) {
    if (!active) return;
    m_last_act_ms = millis();
    m_seg_open = 1;
    m_seg_start = blk_pos - PIM_BLK_W;     /* SOF の立ち上がりと IQ 窓の手前ぶんを含める */
    m_seg_last_act = blk_end;
    m_seg_idle = 0;
    m_hang_blk = hang_blocks();
    return;
  }
  if (active) { m_seg_last_act = blk_end; m_seg_idle = 0; m_last_act_ms = millis(); }
  else        { m_seg_idle++; }

  bool close = false;
  uint32_t end = 0;
  if (m_seg_idle >= m_hang_blk) {
    close = true; end = m_seg_last_act + PIM_BLK_W;
  } else if (blk_end - m_seg_start >= PIM_SEG_N) {
    /* 活動が途切れない（フリーラン送信など）→ 上限で切る。跨いだペアは失う。 */
    close = true; end = m_seg_start + PIM_SEG_N;
    g_pim.truncated++;
  }
  if (!close) return;

  m_seg_open = 0;
  if (seg_decode(m_seg_start, end - m_seg_start) && g_pim.sof_count > 0) {
    uint32_t tp = micros();
    pim_print_result();
    tp = micros() - tp;
    m_st.prt_n++; m_st.prt_sum += tp; if (tp > m_st.prt_max) m_st.prt_max = tp;
    /* ★ここが «全タグ同時に落ちる» への対処。SOF が見えている = タグは送っていて
       磁界もある。それで 1 タグも取れないなら、原因はタグ側ではなく «全タグが
       共有している受信 FE» なので、待たずに入れ直す。 */
    if (g_pim.ntags == 0) {
      if (g_pim.stuck_max && ++m_stuck >= g_pim.stuck_max) { m_resync_req = 1; m_stuck = 0; }
    } else {
      m_stuck = 0;
    }
  }
  if (m_seg_idle < m_hang_blk) {        /* 上限で切った場合は続きを新しいセグメントとして開く */
    m_seg_open = 1; m_seg_start = end; m_seg_last_act = blk_end; m_seg_idle = 0;
  }
}

void pim_listen_task() {
  if (!m_listen_hw) return;
  for (int budget = 256; budget > 0; budget--) {
    uint32_t wr = ring_total();
    g_pim.ring_wr = wr & PIM_RING_MASK;
    uint32_t lag = wr - m_scan_pos;
    if (lag < PIM_BLK_W || lag > 0x80000000u) return;          /* 未着 or 巻き戻り直後 */
    if (lag > m_st.lag_max) m_st.lag_max = lag;
    if (lag > PIM_RING_N - 8u * PIM_BLK_W) {                   /* 追いつけない → 最新側へ */
      m_scan_pos = wr - 4u * PIM_BLK_W;
      m_prev_w = 0xFFFF; m_seg_open = 0; m_seg_idle = 0;
      g_pim.overruns++;
      continue;
    }
    uint32_t t = block_transitions(m_scan_pos);
    m_last_t = t; if (t > m_max_t) m_max_t = t;
    g_pim.blocks++;
    seg_feed(m_scan_pos, t >= g_pim.act_thr);
    m_scan_pos += PIM_BLK_W;
  }
}

/* SWD: g_pim.trigger=1 でリングの最新 PIM_SEG_N word を 1 回復号（表示はしない） */
void pim_poll() {
  if (!g_pim.trigger) return;
  g_pim.trigger = 0;
  if (!m_listen_hw) { g_pim.done++; return; }
  uint32_t wr = ring_total();
  seg_decode(wr - PIM_SEG_N, PIM_SEG_N);
}

/* ===========================================================================
 *  磁界サイクル（検波トリガ式のタグに OFF→ON のエッジを与える）
 *    捕捉は OFF の間だけ止まり、ON にした直後から次のサイクルまで連続で聞く。
 *    タグが ON から何 ms 後に送ってきても捕まる（固定の捕捉窓が無い）。
 * ==========================================================================*/
static void pim_field_cycle_once() {
  m_mute = 1;
  pim_listen_stop();
  pim_transparent_exit();
  pim_field_off();
  delay(g_pim.auto_off_ms ? g_pim.auto_off_ms : 1);
  pim_transparent_enter();                    /* ★ここの立ち上がりがタグのトリガ */
  pim_listen_start();
  m_mute = 0;
}

/* ★磁界を «切らずに» 受信フロントエンドだけ入れ直す（free-run 用）。
 *
 *   pim_field_cycle_once() との違いは «field_off + gap 待ちをしない» ことだけ。
 *   タグは磁界 ON の合図を必要としない（ホストのタイマで撃つ）ので、磁界を落とす
 *   理由が無い。落とさなければ、落ちている間にタグが撃ってしまう事故も起きない。
 *   なお transparent_exit の SET_DEFAULT で送信部も一度既定に戻るため、搬送波は
 *   数 ms だけ途切れる（完全に無瞬断にはできない）。
 *   transparent_enter が pim_rx_config()（RX_CONF1..4 / txpwr / VDD_RF）を
 *   撒き直すので、ここが «受信側の初期化» になる。 */
static void pim_rx_resync_once(const char *why) {
  m_mute = 1;
  pim_listen_stop();
  pim_transparent_exit();     /* SET_DEFAULT: 受信チェーンを既定へ戻す */
  pim_transparent_enter();    /* rx_config（RX_CONF/txpwr/vddrf）+ field ON + TPM */
  pim_listen_start();
  m_mute = 0;
  g_pim.resyncs++;
  /* ★2026-09-28: いつ・なぜ入れ直したかを残す（host が «全滅ラウンド» と突き合わせる）。
     t= は [rx] 行と同じ micros()。1 秒に数行なので UART の負担にはならない。 */
  logf("[resync] why=%s t=%lu\r\n", why, (unsigned long)micros());
}

static uint32_t m_resync_next = 0;     /* 周期 resync の次回時刻 [ms] */

void pim_resync_task() {
  if (!m_listen_hw) { m_resync_req = 0; return; }   /* 聞いていないときは何もしない */
  uint32_t now = millis();
  /* ★詰まり検出による «即» resync。周期が無効(0)でもこちらは効く。 */
  if (m_resync_req) {
    m_resync_req = 0;
    g_pim.stuck_resyncs++;
    pim_rx_resync_once("stuck");
    m_last_act_ms = now;
    m_resync_next = now + (g_pim.resync_ms ? g_pim.resync_ms : 0);  /* 周期も押し戻す */
    return;
  }
  /* ★2026-09-28: «無音» 検出による即 resync。
     入れ直した受信 FE が稀に «何も聞こえない»（ビット遷移ゼロ）状態で上がってくる。
     SOF すら出ないので stuck（SOF>0 なのに tags=0）では検出できず、次の周期 resync
     まで最大 2 s 全タグが落ちていた（50 ms 周期 5 分で 10 回、最長 37 ラウンド）。
     タグが周期送信しているのに quiet_ms 以上活動が無ければ、FE を疑って入れ直す。
     タグがいない運用では意味が無いので既定 0（host の free-run link が設定する）。 */
  if (g_pim.quiet_ms && (uint32_t)(now - m_last_act_ms) > g_pim.quiet_ms) {
    g_pim.quiet_resyncs++;
    pim_rx_resync_once("quiet");
    m_last_act_ms = now;                 /* 直後にまた発火しないよう、次の quiet_ms を待つ */
    return;
  }
  if (!g_pim.resync_ms) return;          /* 0 = 周期 resync は無効（従来動作）*/
  if ((int32_t)(now - m_resync_next) < 0) return;
  m_resync_next = now + g_pim.resync_ms;
  pim_rx_resync_once("periodic");
}

void pim_cycle_task() {
  static uint32_t next_ms = 0;
  if (!g_pim.auto_on) return;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + (g_pim.auto_period_ms ? g_pim.auto_period_ms : 200);
  pim_field_cycle_once();
}

/* ===========================================================================
 *  コンソール
 * ==========================================================================*/
/* レジスタに触る前に必ず呼ぶ。連続捕捉と Transparent を抜けて SPI を戻す。 */
static void pim_need_spi(void) {
  if (m_listen_hw || g_pim.auto_on) {
    pim_listen_stop(); g_pim.listen_on = 0; g_pim.auto_on = 0;
    logf("[listen] stopped for register access ('listen on' to resume)\r\n");
  }
  if (!m_tpm) return;
  logf("[tpm] auto-exit (register access needs SPI pins back)\r\n");
  pim_transparent_exit();
}

static void pim_listen_on_cmd() {
  if (!m_tpm) pim_transparent_enter();
  pim_listen_start();
  uint32_t fs = g_pim.capspi_fs, spsm = sps_milli_eff();
  logf("[listen] ON  fs=%luHz sps=%lu.%03lu ring=%lums\r\n",
       (unsigned long)fs, (unsigned long)(spsm / 1000), (unsigned long)(spsm % 1000),
       (unsigned long)(fs ? (uint64_t)PIM_RING_N * 16u * 1000u / fs : 0));
}

static void pim_stat() {
  uint32_t fs = g_pim.capspi_fs, spsm = sps_milli_eff();
  logf("[stat] listen=%lu cycle=%lu fs=%luHz sps=%lu.%03lu%s ring=%lums\r\n",
       (unsigned long)m_listen_hw, (unsigned long)g_pim.auto_on, (unsigned long)fs,
       (unsigned long)(spsm / 1000), (unsigned long)(spsm % 1000),
       g_pim.sps_milli ? "" : "(auto)",
       (unsigned long)(fs ? (uint64_t)PIM_RING_N * 16u * 1000u / fs : 0));
  logf("[stat] blocks=%lu segments=%lu overruns=%lu truncated=%lu\r\n",
       (unsigned long)g_pim.blocks, (unsigned long)g_pim.segments,
       (unsigned long)g_pim.overruns, (unsigned long)g_pim.truncated);
  logf("[stat] resync=%lums resyncs=%lu  stuck_max=%lu stuck_resyncs=%lu  quiet=%lums quiet_resyncs=%lu\r\n",
       (unsigned long)g_pim.resync_ms, (unsigned long)g_pim.resyncs,
       (unsigned long)g_pim.stuck_max, (unsigned long)g_pim.stuck_resyncs,
       (unsigned long)g_pim.quiet_ms, (unsigned long)g_pim.quiet_resyncs);
  /* ★AGC/Squelch が «本当に» ゲインを下げているかはここでしか分からない。
   *   GAIN_RED_STATE(0x2D) は [7:4]=AM / [3:0]=PM の実減衰段数で、register 指定 +
   *   squelch + AGC の «合計» が出る（DS11793 p.83）。RX_CONF4 の指定値より大きければ
   *   AGC/Squelch が動いている証拠。transparent 中はレジスタに触れないので、
   *   直前の受信 FE 入れ直し時点の値が出る。 */
  logf("[stat] rxconf=%08lX agc=%s squelch=%s gain_red=0x%02X\r\n",
       (unsigned long)g_pim.rxconf,
       (pim_rxconf_byte(1) != 0xFF && (pim_rxconf_byte(1) & 0x10)) ? "on" : "off",
       (pim_rxconf_byte(1) != 0xFF && (pim_rxconf_byte(1) & 0x02)) ? "auto" : "off",
       m_tpm ? 0xFFu : regRead(REG_GAIN_RED_STATE));
  logf("[stat] thr=%lu hang=%lublk mult=%lu last_decode=%luus\r\n",
       (unsigned long)g_pim.act_thr, (unsigned long)hang_blocks(),
       (unsigned long)g_pim.data_tu_mult, (unsigned long)g_pim.decode_us);
  logf("[stat] block transitions: last=%lu max=%lu (max resets here)\r\n",
       (unsigned long)m_last_t, (unsigned long)m_max_t);
  m_max_t = 0;
  /* ★送信周期を詰めるときの見どころ（前回 stat からの集計。ここでリセット）。
       busy = (復号 + 表示) の合計 / 経過時間。100% に近づくと走査が遅れ、lag が
       リング 1 周に届くと overruns になる。 */
  uint32_t el = millis() - m_st.t0_ms; if (!el) el = 1;
  uint32_t busy = (uint32_t)(((uint64_t)m_st.dec_sum + m_st.prt_sum) / 10u / el);   /* % */
  logf("[stat] seg=%lu (%lu/s) len avg=%lu max=%lu sample\r\n",
       (unsigned long)m_st.dec_n, (unsigned long)(m_st.dec_n * 1000u / el),
       (unsigned long)(m_st.dec_n ? m_st.seg_sum / m_st.dec_n : 0), (unsigned long)m_st.seg_max);
  logf("[stat] decode avg=%luus max=%luus  print avg=%luus max=%luus  busy=%lu%%\r\n",
       (unsigned long)(m_st.dec_n ? m_st.dec_sum / m_st.dec_n : 0), (unsigned long)m_st.dec_max,
       (unsigned long)(m_st.prt_n ? m_st.prt_sum / m_st.prt_n : 0), (unsigned long)m_st.prt_max,
       (unsigned long)busy);
  logf("[stat] decode split: scan1=%luus scan2=%luus read_id=%luus (per seg) sof/seg=%lu.%02lu\r\n",
       (unsigned long)(m_st.dec_n ? m_prof[0] / m_st.dec_n : 0),
       (unsigned long)(m_st.dec_n ? m_prof[1] / m_st.dec_n : 0),
       (unsigned long)(m_st.dec_n ? m_prof[2] / m_st.dec_n : 0),
       (unsigned long)(m_st.dec_n ? m_prof[3] / m_st.dec_n : 0),
       (unsigned long)(m_st.dec_n ? (m_prof[3] * 100u / m_st.dec_n) % 100u : 0));
  memset(m_prof, 0, sizeof(m_prof));
  logf("[stat] scan lag max=%lu word (ring %lu = overrun)\r\n",
       (unsigned long)m_st.lag_max, (unsigned long)PIM_RING_N);
  memset(&m_st, 0, sizeof(m_st));
  m_st.t0_ms = millis();
}

static void pim_measure() {
  cmdExec(CMD_MEASURE_AMPLITUDE); delay(10); uint8_t amp = regRead(REG_AD_RESULT);
  cmdExec(CMD_MEASURE_PHASE);     delay(10); uint8_t ph  = regRead(REG_AD_RESULT);
  logf("[meas] RF amplitude=0x%02X  ant phase=0x%02X\r\n", amp, ph);
}
static void pim_dump_registers() {
  logf("[dump] ST25R3911B 0x00..0x3F:\r\n");
  for (uint8_t r = 0; r <= 0x3F; r++)
    logf("  0x%02X=0x%02X%s", r, regRead(r), ((r & 7) == 7) ? "\r\n" : "  ");
  logf("\r\n");
}

/* logf のバッファは 160 byte なので 1 行ずつ出す */
static void pim_help() {
  logf("\r\n==== NFC PIM reader console - 424k RING BUFFER ====\r\n");
  logf("  (continuous capture: packets print as they arrive)\r\n");
  logf("  listen on|off   continuous capture + decode\r\n");
  logf("  cycle on|off    toggle field OFF->ON periodically (alias: auto on|off)\r\n");
  logf("  run             one field OFF->ON now (keeps listening)\r\n");
  logf("  period <ms>     cycle period = tag TX interval (default %u)\r\n",
       (unsigned)PIM_CYCLE_PERIOD_MS);
  logf("  gap <ms>        field-OFF time         (default %u)\r\n",
       (unsigned)PIM_CYCLE_GAP_MS);
  logf("  rate <0..3>     capture 0=40M 1=20M 2=10M(default) 3=5M sps\r\n");
  logf("  thr <n>         activity threshold [transitions/block] (default 4)\r\n");
  logf("  rxconf <hex>    RX_CONF1 (default A0: PM ch + 2MHz LP = best measured;\r\n");
  logf("                  8C = ST 424kHz rec. (600kHz LP) -> splits SOF, do not use;\r\n");
  logf("                  20 = AM ch -> capture doubles in freq, never decodes)\r\n");
  logf("  agc on|off      AGC + auto squelch (RX_CONF2 bit4/bit1; default off = fixed gain)\r\n");
  logf("  gain <0..7>     1st stage PM gain (0=full, 1-6=-2.5dB/step, 7=boost; default 6)\r\n");
  logf("  gain2 <0..10>   2nd/3rd stage PM gain + digitizer window (0=max gain; default 0)\r\n");
  logf("  squelch         run manual squelch once and show 0x2D before/after\r\n");
  logf("  diag <n>        dump thr / SOF amp / ID chip amps for next n segments\r\n");
  logf("  mult <n>        data_tu_mult           (default 1)\r\n");
  logf("  sps auto|<n>    sps x1000, auto = from capture rate\r\n");
  logf("  stat            counters / timing\r\n");
  logf("  resync <ms>|off     re-init RX front-end periodically (field stays ON)\r\n");
  logf("  stuck <n>|off       re-init RX FE at once after n segments of sof>0/tags=0 (def %u)\r\n",
       (unsigned)PIM_STUCK_MAX);
  logf("  quiet <ms>|off      re-init RX FE at once when no activity for <ms> (def off)\r\n");
  logf("  vddrf <0..15>|auto  TX power VDD_RF: 3600+100*rege mV (default %u = %umV)\r\n",
       (unsigned)PIM_VDDRF_REGE, (unsigned)(3600u + 100u * PIM_VDDRF_REGE));
  logf("  bringup | field on|off | tpm | tpx | meas | dump\r\n");
  logf("  r <reg> / w <reg> <val>   read / write register (hex)\r\n");
}

static void pim_exec_line(char *line) {
  while (*line == ' ') line++;
  if (!*line) return;
  unsigned a = 0, b = 0; int si = 0;
  if      (!strcmp(line, "help"))       pim_help();
  else if (!strcmp(line, "listen on"))  pim_listen_on_cmd();
  else if (!strcmp(line, "listen off")) { pim_listen_stop(); g_pim.listen_on = 0; g_pim.auto_on = 0; logf("[listen] OFF\r\n"); }
  else if (!strcmp(line, "cycle on") || !strcmp(line, "auto on")) {
    if (!m_listen_hw) pim_listen_on_cmd();
    g_pim.auto_on = 1;
    logf("[cycle] ON  period=%lums gap=%lums\r\n",
         (unsigned long)g_pim.auto_period_ms, (unsigned long)g_pim.auto_off_ms); }
  else if (!strcmp(line, "cycle off") || !strcmp(line, "auto off")) { g_pim.auto_on = 0; logf("[cycle] OFF (still listening)\r\n"); }
  else if (!strcmp(line, "run"))        { pim_field_cycle_once(); logf("[run] field OFF->ON done, listening\r\n"); }
  else if (!strcmp(line, "stat"))       pim_stat();
  else if (!strcmp(line, "sps auto"))   { g_pim.sps_milli = 0; logf("[sps] auto\r\n"); }
  else if (sscanf(line, "rate %u", &a) == 1) {
    if (a > 3) { logf("? rate 0..3\r\n"); return; }
    bool was = m_listen_hw;
    pim_listen_stop(); g_pim.spi_br = a;
    if (was) pim_listen_start();
    logf("[rate] BR=%u -> %luHz%s\r\n", a,
         (unsigned long)(HAL_RCC_GetPCLK2Freq() / (2u << a)), was ? "" : " (applies on 'listen on')"); }
  else if (sscanf(line, "thr %u", &a) == 1)    { g_pim.act_thr = a;        logf("[thr] %u\r\n", a); }
  /* ★VDD_RF（送信電力）。'vddrf auto' で ADJUST_REGULATORS 任せに戻す。
       手動値は SET_DEFAULT で消えるので、ここでは g_pim に «覚えさせて» から
       すぐ 1 回書く。以後は pim_rx_config() が磁界を入れ直すたびに再適用する。 */
  /* ★受信 FE の定期リセット。free-run（磁界 ON 固定）で «復調が落ちたまま戻らない»
       のを防ぐためのもの。検波トリガ運用では cycle が同じ役目をするので 0 でよい。 */
  else if (!strcmp(line, "stuck off")) {
    g_pim.stuck_max = 0; logf("[stuck] OFF（詰まり検出による自動 resync をしない）\r\n"); }
  else if (sscanf(line, "stuck %u", &a) == 1) {
    g_pim.stuck_max = a;
    logf("[stuck] %u segments -> 即 resync (stuck_resyncs=%lu)\r\n",
         a, (unsigned long)g_pim.stuck_resyncs); }
  else if (!strcmp(line, "quiet off")) {
    g_pim.quiet_ms = 0; logf("[quiet] OFF\r\n"); }
  else if (sscanf(line, "quiet %u", &a) == 1) {
    g_pim.quiet_ms = a; m_last_act_ms = millis();
    logf("[quiet] %ums 活動が無ければ即 resync (quiet_resyncs=%lu)\r\n",
         a, (unsigned long)g_pim.quiet_resyncs); }
  else if (!strcmp(line, "resync off")) {
    g_pim.resync_ms = 0; logf("[resync] OFF\r\n"); }
  else if (sscanf(line, "resync %u", &a) == 1) {
    g_pim.resync_ms = a;
    logf("[resync] every %ums (field は切らない。resyncs=%lu)\r\n",
         a, (unsigned long)g_pim.resyncs); }
  else if (!strcmp(line, "vddrf auto")) {
    pim_need_spi(); g_pim.vddrf_rege = 0xFF;
    regWrite(REG_REGULATOR_CONTROL, 0x00);        /* reg_s=0 = 自動値(reg_d)を使う */
    cmdExec(CMD_ADJUST_REGULATORS);
    logf("[vddrf] auto (ADJUST_REGULATORS)  'listen on' で捕捉を再開すること\r\n"); }
  else if (sscanf(line, "vddrf %u", &a) == 1) {
    if (a > 15) { logf("? vddrf 0..15 (rege) or 'vddrf auto'\r\n"); return; }
    pim_need_spi(); g_pim.vddrf_rege = a; pim_apply_vddrf();
    logf("[vddrf] rege=%u  'listen on' で捕捉を再開すること\r\n", a); }
  else if (!strcmp(line, "agc on") || !strcmp(line, "agc off")) {
    /* RX_CONF2 の agc_en(bit4) と sqm_dyn(bit1) をまとめて切り替える。
     * off にするとゲインは RX_CONF3/4 の値に固定される（`gain` / `gain2` で変える）。 */
    bool on = !strcmp(line, "agc on");           /* "agc on" / "agc off" は line[4] が両方 'o' */
    uint8_t v = pim_rxconf_byte(1);
    if (v == 0xFF) v = PIM_RX_CONF2_PIM;          /* «触らない» だった場合の起点 */
    if (on) v |= (0x10u | 0x02u); else v &= (uint8_t)~(0x10u | 0x02u);
    pim_rxconf_set_byte(1, v);
    logf("[agc] %s -> RX_CONF2=0x%02X (applies on next field cycle)\r\n",
         on ? "on" : "off", v); }
  else if (sscanf(line, "gain %u", &a) == 1) {
    /* RX_CONF3 の 1 段目 PM ゲイン rg1_pm[4:2]: 0=full, 1..6=-2.5dB/step, 7=+5.5dB boost */
    if (a > 7) { logf("? gain 0..7 (0=full, 1-6=-2.5dB/step, 7=boost +5.5dB)\r\n"); return; }
    uint8_t v = pim_rxconf_byte(2);
    if (v == 0xFF) v = PIM_RX_CONF3_PIM;
    v = (uint8_t)((v & ~0x1Cu) | ((a & 7u) << 2));
    pim_rxconf_set_byte(2, v);
    logf("[gain] 1st stage pm=%u -> RX_CONF3=0x%02X (%s)\r\n", a, v,
         (a == 0) ? "full" : (a == 7) ? "boost +5.5dB" : "reduced"); }
  else if (sscanf(line, "gain2 %u", &a) == 1) {
    /* RX_CONF4 の PM 側 rg2_pm[3:0]: 0=最大ゲイン/最小窓、1-4 は digitizer 窓を 3dB ずつ
     * 広げ、5-10 はさらに 2/3 段目を 3dB ずつ下げる。0..10 のみ有効。 */
    if (a > 10) { logf("? gain2 0..10 (0=max gain, 1-4=widen digitizer window, 5-10=also -3dB)\r\n"); return; }
    uint8_t v = pim_rxconf_byte(3);
    if (v == 0xFF) v = PIM_RX_CONF4_PIM;
    v = (uint8_t)((v & ~0x0Fu) | (a & 0x0Fu));
    pim_rxconf_set_byte(3, v);
    logf("[gain2] 2nd/3rd stage pm=%u -> RX_CONF4=0x%02X\r\n", a, v); }
  else if (!strcmp(line, "squelch")) {
    /* 手動 Squelch(0xD4)。rx_on が low のときだけ実行されるので transparent 中は効かない。
     * «AGC が本当に効くのか» を確かめる実験用（撃つ前後で 0x2D を読む）。 */
    pim_need_spi();
    logf("[squelch] red before=0x%02X\r\n", regRead(REG_GAIN_RED_STATE));
    cmdExec(CMD_SQUELCH); delay(1);
    logf("[squelch] red after =0x%02X ('listen on' で再開)\r\n", regRead(REG_GAIN_RED_STATE)); }
  else if (sscanf(line, "rxconf %x", &a) == 1) {
    /* ★RX_CONF1 «だけ» を差し替える。以前は上位 3 byte を 0xFF で潰していたので、
     *   `rxconf` を打つたびに agc/gain/gain2 の設定が «不変» に戻ってしまっていた
     *   （2026-09-28 の A/B 測定でこれに引っかかった）。次の磁界サイクル /
     *   resync での transparent 再突入で適用される。 */
    pim_rxconf_set_byte(0, (uint8_t)(a & 0xFFu));
    logf("[rxconf] RX_CONF1=0x%02X (applies on next field cycle)\r\n", (unsigned)(a & 0xFF)); }
  else if (sscanf(line, "idoff %d", &si) == 1) { m_idoff = si;            logf("[idoff] %d sample\r\n", si); }
  else if (sscanf(line, "diag %u", &a) == 1)   { m_diag = a;              logf("[diag] next %u segment(s)\r\n", a); }
  else if (sscanf(line, "mult %u", &a) == 1)   { g_pim.data_tu_mult = a ? a : 1; logf("[mult] %lu\r\n", (unsigned long)g_pim.data_tu_mult); }
  else if (sscanf(line, "period %u", &a) == 1) { g_pim.auto_period_ms = a; logf("[cycle] period=%ums\r\n", a); }
  else if (sscanf(line, "gap %u", &a) == 1)    { g_pim.auto_off_ms = a;    logf("[cycle] gap=%ums\r\n", a); }
  else if (sscanf(line, "sps %u", &a) == 1) {
    g_pim.sps_milli = a;
    logf("[sps] %u (=%u.%03u)\r\n", a, a/1000, a%1000);
    /* 40 Msps 用の較正値（例 94800）を 10 Msps で使うと復号が全滅するので警告する */
    uint32_t au = g_pim.capspi_fs ? (uint32_t)((double)g_pim.capspi_fs * 1000.0 / PIM_FSC_HZ + 0.5) : 0;
    if (au && (a > au + au / 10 || a + au / 10 < au))
      logf("[sps] WARNING: %luHz capture gives sps~%lu; this value is >10%% off\r\n",
           (unsigned long)g_pim.capspi_fs, (unsigned long)au);
  }
  else if (!strcmp(line, "bringup"))   { pim_need_spi(); pim_bringup(); }
  else if (!strcmp(line, "field on"))  { pim_need_spi(); pim_field_on(); }
  else if (!strcmp(line, "field off")) { pim_need_spi(); pim_field_off(); }
  else if (!strcmp(line, "tpm"))       { pim_need_spi(); pim_transparent_enter(); }
  else if (!strcmp(line, "tpx"))       { pim_listen_stop(); g_pim.listen_on = 0; g_pim.auto_on = 0; pim_transparent_exit(); }
  else if (!strcmp(line, "meas"))      { pim_need_spi(); pim_measure(); }
  else if (!strcmp(line, "dump"))      { pim_need_spi(); pim_dump_registers(); }
  else if (sscanf(line, "w %x %x", &a, &b) == 2) { pim_need_spi(); regWrite((uint8_t)a, (uint8_t)b); logf("[reg] W 0x%02X=0x%02X (rb 0x%02X)\r\n", a, b, regRead((uint8_t)a)); }
  else if (sscanf(line, "r %x", &a) == 1)        { pim_need_spi(); logf("[reg] R 0x%02X=0x%02X\r\n", a, regRead((uint8_t)a)); }
  else logf("? unknown: '%s' (try 'help')\r\n", line);
}

void pim_console_task() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') { logf("\r\n"); buf[len] = 0; pim_exec_line(buf); len = 0; logf("> "); }
    else if (c == 0x08 || c == 0x7F) { if (len) { len--; logf("\b \b"); } }
    else if (len < sizeof(buf) - 1) { buf[len++] = c; char e[2] = { c, 0 }; logf("%s", e); }
  }
}

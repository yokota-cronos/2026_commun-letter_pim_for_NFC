/**
 * pim_ring424k.ino — STM32duino PIM リーダ 424 kHz・リングバッファ版（main 部）。
 *
 *   firmware-multitag/ から派生。変えたのは「捕捉の仕方」で、復号アルゴリズムは同一。
 *
 *   multitag 版（スナップショット方式）:
 *     磁界 OFF→ON のたびに 13.1 ms だけ捕捉して止め、それを復号する。タグの 1 回の
 *     送信がその窓に入らないと取りこぼすので、backoff を窓に合わせる必要があった
 *     （Garment-Coil-Rig/PIM_424kHz_TROUBLESHOOTING.md §2.11）。
 *
 *   この版（リングバッファ方式）:
 *     SPI1+DMA を循環モードで回しっぱなしにし、g_ring[] を常に最新の生波形で上書きする。
 *     メインループが 2 段で追いかける:
 *       1 段目（ブロックごと・軽い）: 64 word ごとにビット遷移数を数え、サブキャリアが
 *                                    出ているかだけを判定する。実時間で追いつける。
 *       2 段目（パケットごと・重い）: 活動が始まってから「最長の間隔より長い無音」が
 *                                    続いたら、その 1 サイクルを g_seg[] に切り出して
 *                                    multitag 版と同じ復号をかける。
 *     決まった窓が無いので、タグがいつ送っても捕まる。
 *
 *   捕捉レートの既定は 10 Msps（sps≈23.6）。40 Msps（sps≈94）では 1 セグメントの
 *   復号がリング 1 周（13.1 ms）と同程度かかり、その間の信号を取りこぼしうるため。
 *   10 Msps ならリング 1 周 52 ms・復号量は 1/4。RESULT.md P-1 の「捕捉レートを下げる」案。
 *   `rate` コマンドで 40 Msps に戻せる（`stat` で復号時間と取りこぼし数を確認できる）。
 *
 *   Board : Nucleo-L476RG + X-NUCLEO-NFC05A1 (ST25R3911B), SYSCLK 80 MHz.
 *   Pins  : SPI1 SCK=PA5(D13) MISO=PA6(D12) MOSI=PA7(D11), CS=PB6(D10, soft), MODE1.
 *           捕捉 SPI1_SCK は PB3(AF5)。MCO PA8=SYSCLK/8=10MHz。
 *           コンソール = Serial(USART2 PA2/PA3 @115200)。
 *   Build/flash: ../build.sh, ../flash.sh
 */
#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
#include <math.h>

/* ===========================================================================
 *  ST25R3911B レジスタレベル SPI アクセス（DS11793 の最小サブセット）
 * ==========================================================================*/
#define PIN_CS   PB6
static SPISettings SPICFG(5000000, MSBFIRST, SPI_MODE1);   /* CPOL=0,CPHA=1 */

static const uint8_t OP_RD = (1 << 6);   /* 0x40 read */
static const uint8_t OP_WR = 0x00;       /* write     */

enum {
  REG_IO_CONF1 = 0x00, REG_IO_CONF2 = 0x01, REG_OP_CONTROL = 0x02, REG_MODE = 0x03,
  REG_RX_CONF1 = 0x0A, REG_RX_CONF2 = 0x0B, REG_RX_CONF3 = 0x0C, REG_RX_CONF4 = 0x0D,
  REG_AD_RESULT = 0x20, REG_RFO_AM_OFF_LEVEL = 0x27, REG_REGULATOR_CONTROL = 0x2A,
  REG_RSSI_RESULT = 0x2C, REG_GAIN_RED_STATE = 0x2D,
  REG_AUX_DISPLAY = 0x30, REG_IC_IDENTITY = 0x3F
};
enum {
  CMD_SET_DEFAULT = 0xC1, CMD_MEASURE_AMPLITUDE = 0xD3, CMD_SQUELCH = 0xD4,
  CMD_RESET_RX_GAIN = 0xD5, CMD_ADJUST_REGULATORS = 0xD6,
  CMD_CALIBRATE_ANTENNA = 0xD8, CMD_MEASURE_PHASE = 0xD9, CMD_TRANSPARENT_MODE = 0xDC,
  CMD_MEASURE_VDD = 0xDF
};
#define OPC_tx_en   (1 << 3)
#define OPC_rx_en   (1 << 6)
#define OPC_en      (1 << 7)
#define AUX_osc_ok  (1 << 4)
#define AUX_tx_on   (1 << 5)
#define AUX_rx_on   (1 << 3)
#define IO2_miso_pd1 (1 << 3)
#define IO2_miso_pd2 (1 << 4)
#define IO2_sup3V    (1 << 7)

/* ===========================================================================
 *  ★タグが «何 ms ごとに» センサ値を送るか = 磁界サイクルの周期
 *
 *  PIM のタグは «磁界 ON の立ち上がり» を合図に送信を始める（検波トリガ）。だから
 *  リーダが磁界を OFF->ON する周期が、そのままタグの送信間隔になる。
 *  ここを変えれば送信間隔が変わる（論文の評価条件は 100 ms 周期）。
 *
 *    PIM_CYCLE_PERIOD_MS : 送信間隔 [ms]。OFF+ON を 1 周期とする
 *    PIM_CYCLE_GAP_MS    : そのうち磁界 OFF の時間 [ms]。ON はこの差 (period-gap)
 *
 *  gap の決め方: 検波出力が «トリガ閾値より下» まで放電する時間が要る。2026-09-22 実測
 *  では磁界を切ってから 0.03 ms で 0.010 V まで落ちるので、20 ms は充分すぎる余裕。
 *  逆に gap を長くしすぎると ON の時間が減り、backoff の大きいタグが窓に入らなくなる。
 *  ★host 側で毎ラウンド乱数を引く場合（--random-data/--random-backoff）は、AD3 への
 *    波形載せ替えに約 84 ms かかるので period を 200 ms 以上にすること。
 *  実行中に変えたいだけならコンソールの `period <ms>` / `gap <ms>`、ホストからは
 *  pim_multitag.py の READER_PERIOD_MS / READER_GAP_MS（= --reader-period/--reader-gap）。
 * ==========================================================================*/
#define PIM_CYCLE_PERIOD_MS  100u   /* ★送信間隔 [ms]（ここを変える） */
#define PIM_CYCLE_GAP_MS      20u   /* そのうち磁界 OFF の時間 [ms]     */

/* ===========================================================================
 *  ★リーダの送信電力 = VDD_RF（RFO ドライバが振る電圧）
 *
 *  既定（従来）は bring-up の ADJUST_REGULATORS(0xD6) に «自動で» 決めさせていた。
 *  自動は「電源電圧から取れる上限」を探して REGULATOR_RESULT(0x2B) の reg_d に結果を
 *  置く（実測の選択値 reg_d=11 = 4700 mV）。
 *  ここを手動固定にするには REGULATOR_CONTROL(0x2A) に reg_s(bit7)=1 と rege[6:3] を
 *  書く。換算は RFAL と同じで（host/pim_multitag.py の vddrf_mv と同一）:
 *      5 V 電源 (sup3V=0): VDD_RF[mV] = 3600 + 100 * rege
 *      3.3 V 電源(sup3V=1): VDD_RF[mV] = 2400 + 100 * (rege-5)   ← 5 V は出せない
 *
 *    rege = 14 -> 5000 mV = ★5 V 設定（これを既定にした）
 *    rege = 11 -> 4700 mV （自動が選んでいた値）
 *    rege = 15 -> 5100 mV （レジスタ上の最大）
 *
 *  ★実測の注意（2026-09-22, メアンダコイル服）: VDD_RF を 3600->5100 mV に振っても
 *    アンテナ端は 0.051->0.060 Vpp までしか増えない（効き幅 20 %）。しかも rege>=12 は
 *    電源 5.13 V の側で頭打ちになる（だから自動は 11 を選ぶ）。つまり 5 V 固定にしても
 *    «自動より劇的に強くなる» ことは無い。出力を «下げる» なら 0x27(txpwr) が効く。
 *  ★SET_DEFAULT(0xC1) でレジスタは既定へ戻るので、Transparent に入り直すたびに
 *    書き直す必要がある -> pim_rx_config() で毎回適用している（txpwr と同じ扱い）。
 *  実行中に変えるならコンソールの `vddrf <rege>` / `vddrf auto`。
 * ==========================================================================*/
#define PIM_VDDRF_REGE       14u    /* ★5 V 設定 (3600+100*14 = 5000 mV)。0xFF = 自動 */

/* ===========================================================================
 *  ★受信フロントエンドの «定期リセット»（free-run 運用で必要になった）
 *
 *  従来の検波トリガ運用では `cycle on` が period ごとに
 *      transparent_exit(= SET_DEFAULT) -> field OFF -> field ON -> transparent_enter
 *  を回していた。これはタグへ «磁界 ON の合図» を送るためのものだったが、同時に
 *  «受信チェーンを毎周期まっさらに初期化する» 役目も持っていた。
 *
 *  host の --free-run は磁界 ON 固定にしてこのサイクルを止めたので、初期化も
 *  一緒に消えた。その結果、受信側が一度おかしな状態に入ると «二度と戻らない»。
 *  2026-09-25 のログ log/20260925_152819 がまさにこれで、3 タグが 174 ラウンド
 *  （35 秒）まで 100% 復号できていたのに、175 ラウンド目以降は弱い 2 タグだけが
 *  «SOF は見えるのに ID が読めない» 状態に落ち、最後まで復帰しなかった。
 *
 *  そこで磁界は «切らずに» 受信側だけ入れ直す（SET_DEFAULT -> RX 再設定 ->
 *  Transparent 再突入 -> 連続捕捉の再開）。所要は数 ms で、その間だけ聞き逃す。
 *    PIM_RESYNC_MS : 何 ms ごとに入れ直すか。0 = しない（従来動作）
 *  実行中はコンソールの `resync <ms>` / `resync off`、host は --reader-resync。
 * ==========================================================================*/
#define PIM_RESYNC_MS        0u     /* 既定 0 = 無効。free-run では host が設定する */
/* ★«詰まり» を検出したら即 受信 FE を入れ直す（周期 resync を待たない）。
 *   実測 2026-09-25: 受信 FE が «SOF は見えるが ID が取れない» 状態に落ちると、
 *   次の resync まで戻らない。4 タグは同じ FE を共有しているので、落ちた瞬間から
 *   全タグが同時に届かなくなる（1 ラウンド = 全滅が数ラウンド続く）。
 *   周期 resync だけで直そうとすると 400ms 間隔まで詰める必要があり（失敗率
 *   17%@off -> 13%@2000ms -> 5%@400ms）、そのぶん «磁界を入れ直す死角» が増える。
 *   連続 N セグメント失敗したら «その場で» 入れ直すほうが、死角を増やさずに済む。 */
/*   実測 2026-09-25（4 タグ / 40 s / resync 2000ms 併用）:
 *       stuck  失敗フレーム  連続全滅の最大  一致
 *         off     19 %            15         569
 *          3       6 %             6         662
 *          2       4 %             5         673
 *          1       2 %             3         714   <- 既定
 *     1 = «取れないセグメントを 1 個見たら即» でも過剰にはならない（40 s で 36 回）。*/
#define PIM_STUCK_MAX        1u

/* PIM 物理層パラメータ（host/pim.py と同じ値） */
#include "pim_phy.h"
/* RX_CONF1(0x0A) = 受信チャネル選択(bit7) + 復調器方式(bit6) + 帯域(bit5:0)。
 *   ビット配置（DS11793 Rev9 p.61 Table 36）:
 *     bit7 ch_sel  : 0 = AM チャネル / 1 = PM チャネル
 *     bit6 amd_sel : 0 = ピーク検波 / 1 = ミキサ（AM 復調器の方式）
 *     bit5:3 lp    : 2 段目ローパスの -1dB 点（p.9 Table 2）
 *                    000=1200k / 001=600k / 010=300k / 100=2M / 101=7M
 *     bit2:0 h200/h80/z12k : 1 段目と 3 段目のハイパス零点（p.8 Table 1）
 *                    000 -> 1段 60k / 3段 400k     100 -> 1段 60k / 3段 200k
 *
 * ★2026-09-22 実機実測（メアンダコイル服 + AD3 タグ, DIO2）: 受信チャネルは PM 側を使う。
 *     bit7=0 = AM (0x20/0x28/0x60): タグ 423.7kHz に対し «捕捉列が 844.6kHz» = ちょうど 2 倍。
 *       負荷変調が振幅でなく位相側に乗ると、この復調は |cos| 特性になって周波数が倍になる。
 *       こうなると SOF 長も 2 倍に化けるので、sps をどう選んでも復号は成立しない
 *       （sof は立つのに ID だけ永久に読めない、という症状になる）。
 *     bit7=1 = PM (0xA0/0xA8/0x80/0xE0): 捕捉列 422.2kHz = タグと 1:1。link で一致 119 / 誤り 0。
 *
 * ★2026-09-27 に «ST の推奨値» 0x8C (lp=001/600kHz + h200=1) を試したが、2026-09-28 の
 *   実機測定で «明確に悪い» と分かったので 0xA0 に戻した。1 タグ/100ms/250 送信を 3 回
 *   交互に測った結果:
 *       0xA0 (2MHz LP, h200=0): 復号 140/146/145 / 150   sof/frame = 2.00（正しい）
 *       0x8C (600kHz LP, h200=1): 復号   4/  0/  0 / 150   sof/frame = 3.2-3.8（分裂）
 *   機序: sof/frame が 2 から 3.7 に増える = «solid-ON の SOF が複数に割れている»。
 *   600kHz ではサブキャリアの «変調帯域» が通らない。PIM は 1 チップ = 1 TU =
 *   サブキャリア 1 周期なので、占有帯域は 424kHz ± 424kHz ≒ 850kHz まで広がる。
 *   ★DS11793 p.9 Table 3 の "Recommended for 424/484 kHz subcarrier" は «framing 経路»
 *     の推奨で、そこではサブキャリアはもっと緩やかに変調される（ISO14443 は fc/128 等）。
 *     Transparent で «デジタイズ済みサブキャリアそのもの» を捕捉する PIM には当てはまらない。
 *     この用途では 2MHz LP (lp=100) を選ぶのが正しい。
 * 結合・同調・コイルを変えたら «測り直す» こと（`rxconf <hex>` で切り替えて sof と一致率を見る）。 */
#define PIM_RX_CONF1_PIM     0xA0u   /* PM ch + 2MHz LP（実測でこれが最良）★既定           */
#define PIM_RX_CONF1_ST_424  0x8Cu   /* ST の 424kHz 推奨 (600kHz LP + h200)。PIM には不適   */
#define PIM_RX_CONF1_AM      0x20u   /* AM ch（bit7=0）。この環境では 2 倍化して復号できない */

/* ---------------------------------------------------------------------------
 *  AGC / Squelch / ゲイン（RX_CONF2..4） — ★2026-09-28 追加
 *
 *  なぜ «明示的に» 設定するのか
 *    ST25R3911B の AGC と Squelch は «framing が動いている» 前提の機構で、
 *    どちらも «ゲインを下げる» 方向にしか働かない（DS11793 p.9-10）:
 *      - Squelch : 50us の間に digitizer 出力の遷移が 2 回を超えたら 3dB 下げる、
 *                  を «2 回以下になるまで» 繰り返す。424kHz サブキャリアは 50us に
 *                  約 21 遷移あるので、もし走れば «常に最大減衰» まで落ちる。
 *      - AGC     : 受信期間（framing が決める）にゲインを下げる。
 *    Transparent では framing を通さないので、どちらも起動しない «はず» ではある:
 *      Squelch(D4) は「rx_on が low のときだけ実行」で、こちらは rx_on=high 固定
 *      (tpm_pins_observe)。sqm_dyn も「Tx 終了の 18.88us 後に開始」= framing イベント。
 *    ただし «はず» に頼ると «受信 FE の詰まり»（sof>0/tags=0、無音落ち）の原因候補が
 *    ひとつ残り続ける。そこで既定で AGC/自動 Squelch を «切って» ゲインを固定し、
 *    実際に減衰が起きていないかを GAIN_RED_STATE(0x2D) で «読んで確かめる» ことにした。
 *    `stat` が 0x2D を表示する。0 以外が出たら AGC/Squelch が本当に効いている証拠。
 *
 *  RX_CONF2(0x0B) bit4 agc_en / bit1 sqm_dyn を 0 にする。他ビットはチップ既定のまま。
 *    チップ既定 0x1A = agc_en(1)+agc_m(1)+sqm_dyn(1)。ここでは 0x08 = agc_m だけ残す
 *    （agc_en=0 なら agc_m は無意味だが、既定との差分を bit2 つに絞って読みやすくする）。
 *
 *  RX_CONF3(0x0C) = 1 段目ゲイン。★チップ既定は «最小ゲイン»（DS11793 p.7）。
 *    bits[7:5] rg1_am / bits[4:2] rg1_pm: 0=フルゲイン, 1-6=2.5dB/step 減衰, 7=+5.5dB boost。
 *    既定 0xD8 は am=6/pm=6 = 両チャネル -15dB。PM チャネルを使うので rg1_pm が効く。
 *    コンソール `gain <0..7>` で変えられる（0=フル, 7=boost, 既定の 6=最小）。
 *
 *  RX_CONF4(0x0D) = 2/3 段目ゲイン + digitizer 窓。0x00 = 最大ゲイン・最小窓（チップ既定）。
 *    bits[3:0] が PM 側で 0..0xA のみ有効: 1-4 は digitizer 窓を 3dB ずつ広げる、
 *    5-A はさらに 2/3 段目を 3dB ずつ下げる。コンソール `gain2 <0..10>`。
 *    ★書いたあとに RESET_RX_GAIN(0xD5) を送らないと AGC/Squelch/RSSI に読み込まれない
 *      （DS11793 p.63 注 2）。pim_rx_config() が毎回送る。
 * --------------------------------------------------------------------------*/
#define PIM_RX_CONF2_PIM     0x08u   /* AGC off (bit4=0) + 自動 Squelch off (bit1=0)        */
#define PIM_RX_CONF3_PIM     0xD8u   /* 1 段目: am=-15dB / pm=-15dB（チップ既定を明示）      */
#define PIM_RX_CONF4_PIM     0x00u   /* 2/3 段目 最大ゲイン + digitizer 窓 最小（チップ既定） */

/* g_pim.rxconf にパックした形（[7:0]=CONF1 .. [31:24]=CONF4、0xFF のバイトは書かない） */
#define PIM_RXCONF_PACKED  (((uint32_t)PIM_RX_CONF4_PIM << 24) | \
                            ((uint32_t)PIM_RX_CONF3_PIM << 16) | \
                            ((uint32_t)PIM_RX_CONF2_PIM <<  8) | \
                             (uint32_t)PIM_RX_CONF1_PIM)

static inline void csLow()  { digitalWrite(PIN_CS, LOW);  }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

uint8_t regRead(uint8_t reg) {
  SPI.beginTransaction(SPICFG); csLow();
  SPI.transfer(reg | OP_RD); uint8_t v = SPI.transfer(0x00);
  csHigh(); SPI.endTransaction(); return v;
}
void regWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPICFG); csLow();
  SPI.transfer(reg | OP_WR); SPI.transfer(val);
  csHigh(); SPI.endTransaction();
}
void cmdExec(uint8_t c) {
  SPI.beginTransaction(SPICFG); csLow(); SPI.transfer(c); csHigh(); SPI.endTransaction();
}
void regSet(uint8_t reg, uint8_t m) { regWrite(reg, regRead(reg) |  m); }
void regClr(uint8_t reg, uint8_t m) { regWrite(reg, regRead(reg) & ~m); }
void regChange(uint8_t reg, uint8_t mask, uint8_t val) {
  regWrite(reg, (regRead(reg) & ~mask) | (val & mask));
}

/* printf over VCP */
static char _lb[160];
void logf(const char *fmt, ...) {
  va_list ap; va_start(ap, fmt); vsnprintf(_lb, sizeof(_lb), fmt, ap); va_end(ap);
  Serial.print(_lb);
}

/* ===========================================================================
 *  バッファ寸法
 * ==========================================================================*/
#define PIM_RING_N    32768u              /* リング [word]。2 のべき乗必須（マスクで回す） */
#define PIM_RING_MASK (PIM_RING_N - 1u)
#define PIM_SEG_N     4096u               /* 復号用に切り出す 1 セグメントの上限 [word] */
#define PIM_BLK_W     64u                 /* 1 段目の判定単位 [word] = 1024 sample     */

/* ===========================================================================
 *  SWD 共有 I/O
 *    [0]..[15] は firmware-multitag と同じ並び（host が offset で読む）。[16] 以降が追加分。
 * ==========================================================================*/
typedef struct {
  volatile uint32_t trigger;       /* [0]  1 = リングの最新 PIM_SEG_N word を 1 回復号     */
  volatile uint32_t sps_milli;     /* [1]  サンプル/TU ×1000。0 = capspi_fs から自動算出   */
  volatile uint32_t data_tu_mult;  /* [2]  value→interval 倍率                            */
  volatile uint32_t rxconf;        /* [3]  [7:0]=RX_CONF1..[31:24]=RX_CONF4, 0xFF=不変     */
  volatile uint32_t txpwr;         /* [4]  RFO_AM_OFF_LEVEL (0xFF=既定)                   */
  volatile uint32_t done;          /* [5]  復号したセグメントごとに +1                     */
  volatile uint32_t valid;         /* [6]  1 = 直近のセグメントでペアが成立                */
  volatile uint32_t tag_id;        /* [7]                                                 */
  volatile uint32_t data;          /* [8]                                                 */
  volatile int32_t  interval_tu;   /* [9]                                                 */
  volatile uint32_t sof_count;     /* [10] 直近セグメントの SOF 数                         */
  volatile uint32_t capspi_fs;     /* [11] 実捕捉レート [Hz]                               */
  volatile uint32_t ntags;         /* [12] 直近セグメントで復号できたタグ数                */
  volatile uint32_t auto_on;       /* [13] 1 = 磁界サイクル（OFF→ON を周期的に作る）       */
  volatile uint32_t auto_off_ms;   /* [14] 磁界 OFF 時間 [ms]                              */
  volatile uint32_t auto_period_ms;/* [15] 磁界サイクル周期 [ms]                           */
  /* ---- リングバッファ版の追加 ---- */
  volatile uint32_t listen_on;     /* [16] 1 = 連続捕捉中                                  */
  volatile uint32_t spi_br;        /* [17] SPI1 BR: 0=40M 1=20M 2=10M 3=5M [sps]           */
  volatile uint32_t act_thr;       /* [18] 1 段目の活動しきい値 [遷移/ブロック]            */
  volatile uint32_t blocks;        /* [19] 1 段目が走査したブロック数                      */
  volatile uint32_t segments;      /* [20] 2 段目が復号したセグメント数                    */
  volatile uint32_t overruns;      /* [21] 走査が追いつかず読み飛ばした回数                */
  volatile uint32_t truncated;     /* [22] PIM_SEG_N を超えて途中で切ったセグメント数      */
  volatile uint32_t ring_wr;       /* [23] DMA の書き込み位置 [word]                       */
  volatile uint32_t decode_us;     /* [24] 直近セグメントの復号時間 [us]                   */
  volatile uint32_t vddrf_rege;    /* [25] VDD_RF の rege[6:3] (0xFF=自動/ADJUST_REGULATORS)*/
  volatile uint32_t resync_ms;     /* [26] 受信 FE を入れ直す周期 [ms]（0=しない）         */
  volatile uint32_t resyncs;       /* [27] 入れ直した回数                                  */
  volatile uint32_t stuck_max;     /* [28] «SOF は見えるのに 1 タグも取れない» が何セグメント
                                          続いたら即 resync するか（0 = 自動 resync しない）*/
  volatile uint32_t stuck_resyncs; /* [29] その «詰まり検出» で入れ直した回数              */
  volatile uint32_t quiet_ms;      /* [30] 活動が何 ms 途絶えたら受信 FE を入れ直すか（0=しない）*/
  volatile uint32_t quiet_resyncs; /* [31] その «無音検出» で入れ直した回数                */
} pim_io_t;

#define PIM_MAXTAGS 16
typedef struct {
  volatile uint32_t tag_id;
  volatile uint32_t data;
  volatile int32_t  interval_tu;
  volatile uint32_t t_us;          /* preamble 先頭の時刻 [us, micros() 基準]。host がラウンドを
                                      «届いた時刻» ではなく «送られた時刻» で対応付けるのに使う */
} pim_tag_t;
pim_tag_t g_tags[PIM_MAXTAGS];

volatile pim_io_t g_pim = {
  0, 0, 1, PIM_RXCONF_PACKED, 0xFFu,
  0, 0, 0, 0, 0, 0, 0,        /* 既定: RX_CONF1=0xA0, CONF2=0x08(AGC off), CONF3=0xD8, CONF4=0x00 */
  0, 0, PIM_CYCLE_GAP_MS, PIM_CYCLE_PERIOD_MS,        /* ntags, cycle, off_ms, period */
  0, 2, 4, 0, 0, 0, 0, 0, 0,                          /* listen, BR=10Msps, thr=4    */
  PIM_VDDRF_REGE,                                     /* vddrf_rege = 5V 固定        */
  PIM_RESYNC_MS, 0,                                   /* resync_ms, resyncs          */
  PIM_STUCK_MAX, 0,                                   /* stuck_max, stuck_resyncs    */
  0, 0                                                /* quiet_ms, quiet_resyncs     */
};
uint16_t          g_ring[PIM_RING_N];   /* DMA が循環で書き続ける生波形（MSB first）     */
uint16_t          g_seg[PIM_SEG_N];     /* 2 段目が復号する 1 セグメント（線形に並べ直し） */
volatile uint32_t g_diag[4]  = {0};     /* [0]='PIM ' [1]=(chipOk<<24|id<<16|rev<<8|osc<<1) */
volatile uint32_t g_heartbeat = 0;

/* pim.ino の入口 */
bool pim_bringup();
void pim_transparent_enter();
void pim_capture_init();
void pim_listen_start();
void pim_console_task();
void pim_poll();
void pim_listen_task();
void pim_cycle_task();
void pim_resync_task();

/* ===========================================================================
 *  Clock: HSI16 → PLL → 80 MHz（multitag 版と同一）。MCO(PA8)=SYSCLK/8=10MHz。
 *  捕捉 SPI1 は PCLK2(80MHz)/2^(BR+1)。
 * ==========================================================================*/
extern "C" void SystemClock_Config(void)
{
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitTypeDef o = {0};
  o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = 16;
  o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  o.PLL.PLLM = 1; o.PLL.PLLN = 10;
  o.PLL.PLLP = RCC_PLLP_DIV7; o.PLL.PLLQ = RCC_PLLQ_DIV2; o.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&o);
  RCC_ClkInitTypeDef c = {0};
  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV1; c.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&c, FLASH_LATENCY_4);
  SystemCoreClockUpdate();
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);
  GPIOA->OSPEEDR |= (3u << (8 * 2));
}

/* ===========================================================================
 *  Arduino setup / loop
 * ==========================================================================*/
void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT); csHigh();
  SPI.begin();
  pinMode(PA0, INPUT);            /* A0 は既定アナログ → デジタル入力に（IRQ ピン） */
  EXTI->IMR1 &= ~(1u << 0);       /* PA0 EXTI 無効化（IRQ 洪水を防ぐ）              */

  logf("\r\n============================================\r\n");
  logf(" NFC PIM reader - 424kHz RING BUFFER\r\n");
  logf(" Nucleo-L476RG + ST25R3911B, continuous capture\r\n");
  logf("============================================\r\n");

  pim_capture_init();
  bool ok = pim_bringup();
  if (ok) {
    pim_transparent_enter();
    pim_listen_start();
    logf("[init] field ON + Transparent + continuous capture.\r\n");
    logf("[init] packets are printed as they arrive. 'cycle on' = toggle field for detector tags.\r\n");
  } else {
    logf("Bring-up failed. Fix wiring/power, then send 'bringup'.\r\n");
  }
  logf("Type 'help' and press Enter.\r\n> ");
}

void loop() {
  pim_listen_task();      /* 1 段目の走査 → 必要なら 2 段目の復号・表示   */
  pim_console_task();     /* UART コンソール                              */
  pim_poll();             /* SWD: g_pim.trigger=1 でリング最新部分を復号   */
  pim_cycle_task();       /* 'cycle on' 中: 磁界 OFF→ON を周期的に作る     */
  pim_resync_task();      /* 'resync <ms>' 中: 磁界は切らず受信だけ入れ直す */
  g_heartbeat++;
}

/* ===========================================================================
 *  pim_phy.h - PIM 物理層パラメータ
 *
 *  host/pim.py（物理層の唯一の真実）の物理層定義から生成した値。
 *
 *  値を変えたいときは host/pim.py と両方を同じ値に直すこと。片方だけ直すと
 *  ホストとファームが食い違い、loopback 自己テストは通るのに実機で壊れる。
 *
 *  チップ依存の値（RX_CONF1 等）はここには入らない。物理層ではなく
 *  ST25R3911B / ST25R3916B の都合で決まる値なので、チップ層に置くこと。
 * =========================================================================*/
#ifndef PIM_PHY_H
#define PIM_PHY_H

/* ホストとファームの物理層が一致しているかを実行時に照合するための識別子。 */
#define PIM_PHY_SIGNATURE    "fc32-manch-sof4_8-id8"

/* ---- 搬送波とサブキャリア ---- */
#define PIM_FC_HZ            13560000.0f      /* NFC 搬送波 */
#define PIM_SUBCARRIER_DIV   32         /* サブキャリア = fc/32 */
#define PIM_FSC_HZ           423750.0000f  /* = fc / 32 */
#define PIM_TU_US            2.359882f   /* 1 TU = 1/FSC [us] */

/* ---- 符号化 ---- */
#define PIM_CHIPS_PER_BIT    2         /* Manchester: 1 bit = 2 chip = 2 TU */
#define PIM_ID_BITS          8         /* Tag ID = 1 byte */
#define PIM_ID_TU            16        /* = ID_BITS * CHIPS_PER_BIT */

/* ---- SOF: 長さで preamble/postamble を区別する（delimiter バイトは持たない）
 *   Manchester の ON 連続は最大 2 TU なので、>=SOF_MIN_TU の連続 ON は SOF だけ。
 *   ★SOF_MIN_TU と SOF_CLASS_TU は «Manchester であること» から導出された値。
 *     符号化方式を変えるときは pim.py 側で導出をやり直すこと。 ---- */
#define PIM_SOF_PRE_TU       4         /* preamble の SOF 長（短い） */
#define PIM_SOF_POST_TU      8         /* postamble の SOF 長（長い） */
#define PIM_SOF_MIN_TU       3         /* SOF とみなす最小 ON 連続 */
#define PIM_SOF_CLASS_TU     6.0f      /* 種別しきい値 = (PRE+POST)/2 */
#define PIM_GAP_TU           1         /* SOF 直後の OFF */

/* ---- パルス長 [TU]（間隔＝値 の基準）---- */
#define PIM_PRE_PULSE_TU     21        /* = SOF_PRE  + GAP + ID_TU */
#define PIM_POST_PULSE_TU    25        /* = SOF_POST + GAP + ID_TU */

/* ---- 値→間隔 の倍率（既定値。実行時は g_pim.data_tu_mult が優先）---- */
#define PIM_DATA_TU_MULT_DEFAULT 1

/* ---- 復号（受信機）側の判定パラメータ ----
 *   物理層の «定義» ではないが、ホストとファームで «同じ判定» をしないと
 *   loopback では通るのに実機だけ落ちる、という事故が起きるので一緒に生成する。
 *   ---- */
#define PIM_ID_CHIP_CONTRAST 0.25f      /* ビット内 2 チップの最小コントラスト x 閾値 */
#define PIM_ID_SEARCH_CHIPS  0.50f      /* ID サンプル位置の探索幅 [chip] */

/* ---- パルス種別 ---- */
#define PIM_KIND_PREAMBLE    0
#define PIM_KIND_POSTAMBLE   1

#endif /* PIM_PHY_H */

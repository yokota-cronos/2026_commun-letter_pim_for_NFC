/**
 * ring_isr.cpp — DMA1_Channel2 (SPI1_RX) の転送完了割り込み = リング 1 周ごと。
 *
 * .ino に置くと arduino-builder が C++ リンケージのプロトタイプを自動生成し、
 * extern "C" の定義と衝突してビルドが通らない。.cpp はプロトタイプ生成の対象外なので、
 * この割り込みだけここに分けている。
 */
#include <Arduino.h>

volatile uint32_t g_ring_wraps = 0;

extern "C" void DMA1_Channel2_IRQHandler(void) {
  if (DMA1->ISR & DMA_ISR_TCIF2) g_ring_wraps++;
  DMA1->IFCR = DMA_IFCR_CGIF2;
}

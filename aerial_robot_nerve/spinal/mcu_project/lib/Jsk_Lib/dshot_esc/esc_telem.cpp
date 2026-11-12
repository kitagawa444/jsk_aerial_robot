//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

void ESCReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  esc_telem_rd_ptr_ = 0;
  parser_.reset();
  memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);
  HAL_UART_Receive_DMA(huart, esc_telem_rx_buf_, ESC_BUFFER_SIZE);
  // DMA setup enables error interrupts; disable them after starting reception.
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
}

void ESCReader::beginRequest()
{
  esc_telem_rd_ptr_ = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % ESC_BUFFER_SIZE;
  parser_.reset();
}

bool ESCReader::update(spinal::ESCTelemetry& esc_msg)
{
  if (huart_->Instance->ISR & (UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    beginRequest();
    return false;
  }

  // Byte 0: Temperature
  // Byte 1: Voltage high byte
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte
  // Byte 6: Consumption low byte
  // Byte 7: Rpm high byte
  // Byte 8: Rpm low byte
  // Byte 9: 8-bit CRC

  uint8_t buffer[10];  // buffer for KISS esc telemetry data

  bool complete = false;
  for (unsigned i = 0; i < ESC_BUFFER_SIZE; ++i)
  {
    const int data = readOneByte();
    if (data < 0) break;
    complete = parser_.push(static_cast<uint8_t>(data), buffer);
    if (complete) break;
  }
  if (!complete) return false;

  /* check crc */
  uint8_t crc = get_crc8(buffer, 9);
  if (crc == buffer[9])  // crc error
  {
    /* save data in esc_msg_1_ */
    esc_msg.temperature = buffer[0];
    esc_msg.voltage = buffer[1] << 8 | buffer[2];
    esc_msg.current = buffer[3] << 8 | buffer[4];
    esc_msg.consumption = buffer[5] << 8 | buffer[6];
    uint16_t erpm = buffer[7] << 8 | buffer[8];
    esc_msg.rpm = num_motor_mag_pole_ >= 2 ? erpm * 100 / (num_motor_mag_pole_ / 2) : 0;
  }
  esc_msg.crc_error = 0;
  return true;
}

bool ESCReader::available()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  return (esc_telem_rd_ptr_ != dma_write_ptr);
}

int ESCReader::readOneByte()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  int c = -1;
  if (esc_telem_rd_ptr_ != dma_write_ptr)
  {
    c = (int)esc_telem_rx_buf_[esc_telem_rd_ptr_++];
    esc_telem_rd_ptr_ %= ESC_BUFFER_SIZE;
  }
  return c;
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}

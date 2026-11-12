//
// Created by li-jinjie on 24-1-21.
// ref: https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main
//

#include "dshot.h"

void DShot::init(dshot_type_e dshot_type, TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
                 TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2, TIM_HandleTypeDef* htim_motor_3,
                 uint32_t channel_motor_3, TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4)
{
  htim_motor_1_ = htim_motor_1;
  channel_motor_1_ = channel_motor_1;
  htim_motor_2_ = htim_motor_2;
  channel_motor_2_ = channel_motor_2;
  htim_motor_3_ = htim_motor_3;
  channel_motor_3_ = channel_motor_3;
  htim_motor_4_ = htim_motor_4;
  channel_motor_4_ = channel_motor_4;

  dshot_set_timer(dshot_type);
  dshot_put_tc_callback_function();
  dshot_start_pwm();
}

void DShot::initTelemetry(UART_HandleTypeDef* huart)
{
  esc_reader_.init(huart);
  is_telemetry_ = true;
  esc_reader_.num_motor_mag_pole_ = 14; // TODO: should be set by onboard PC
  for (unsigned i = 0; i < 4; ++i) telemetry(i).crc_error = 255;
}

spinal::ESCTelemetry& DShot::telemetry(unsigned motor)
{
  switch (motor)
  {
    case 0: return esc_reader_.esc_msg_1_;
    case 1: return esc_reader_.esc_msg_2_;
    case 2: return esc_reader_.esc_msg_3_;
    default: return esc_reader_.esc_msg_4_;
  }
}

bool DShot::telemetryFresh() const
{
  if (received_mask_ != 15) return false;
  if (esc_reader_.esc_msg_1_.crc_error || esc_reader_.esc_msg_2_.crc_error ||
      esc_reader_.esc_msg_3_.crc_error || esc_reader_.esc_msg_4_.crc_error) return false;
  const uint32_t now = HAL_GetTick();
  for (unsigned i = 0; i < 4; ++i)
    if (now - last_received_ms_[i] > 250) return false;
  return true;
}

void DShot::write(uint16_t* motor_value_array, bool is_telemetry)
{
  // Never rewrite a buffer while its previous transfer is in progress.
  if (htim_motor_1_->hdma[TIM_DMA_ID_CC1]->State != HAL_DMA_STATE_READY ||
      htim_motor_2_->hdma[TIM_DMA_ID_CC2]->State != HAL_DMA_STATE_READY ||
      htim_motor_3_->hdma[TIM_DMA_ID_CC3]->State != HAL_DMA_STATE_READY ||
      htim_motor_4_->hdma[TIM_DMA_ID_CC4]->State != HAL_DMA_STATE_READY)
  {
    return;
  }
  const uint32_t now = HAL_GetTick();
  bool is_telemetry_array[4] = {false, false, false, false};
  if (!transmission_started_)
  {
    ready_after_ms_ = now;
    transmission_started_ = true;
  }
  if (is_telemetry)
  {
    if (id_telem_prev_ >= 0)
    {
      const unsigned motor = id_telem_prev_;
      if (esc_reader_.update(telemetry(motor)))
      {
        last_received_ms_[motor] = now;
        received_mask_ |= 1U << motor;
        id_telem_prev_ = -1;
        if (motor == 3) esc_reader_.is_update_all_msg_ = true;
      }
      else if (now - last_request_ms_ >= 5)
      {
        telemetry(motor).crc_error = 255; // no valid reply, not a CRC-difference counter
        id_telem_prev_ = -1;
        last_request_ms_ = now; // leave a quiet gap before changing motor
        if (motor == 3) esc_reader_.is_update_all_msg_ = true;
      }
    }
    for (unsigned i = 0; i < 4; ++i)
    {
      if (!(received_mask_ & (1U << i)) || now - last_received_ms_[i] > 250)
      {
        spinal::ESCTelemetry& msg = telemetry(i);
        msg.temperature = msg.voltage = msg.current = msg.consumption = msg.rpm = 0;
        msg.crc_error = 255;
        received_mask_ &= ~(1U << i);
      }
    }
    if (id_telem_prev_ < 0 && now - last_request_ms_ >= 2)
    {
      esc_reader_.beginRequest();
      is_telemetry_array[id_telem_] = true;
      id_telem_prev_ = id_telem_;
      id_telem_ = (id_telem_ + 1) % 4;
      last_request_ms_ = now;
    }
  }
  uint16_t values[4];
  for (unsigned i = 0; i < 4; ++i)
  {
    // Give the ESC a zero-throttle startup interval, even if a test was latched.
    values[i] = now - ready_after_ms_ < 1500 ? 0 : motor_value_array[i];
  }
  // send dshot signal
  dshot_prepare_dmabuffer_all(values, is_telemetry_array);
  if (dshot_dma_start()) dshot_enable_dma_request();
  else id_telem_prev_ = -1;
}

/* Static functions */
uint32_t DShot::dshot_choose_type(dshot_type_e dshot_type)
{
  switch (dshot_type)
  {
    case (DSHOT600):
      return DSHOT600_HZ;

    case (DSHOT300):
      return DSHOT300_HZ;

    case (DSHOT150):
      return DSHOT150_HZ;

    default:
      return DSHOT300_HZ;
  }
}

void DShot::dshot_set_timer(dshot_type_e dshot_type)
{
  TIM_HandleTypeDef* timers[4] = {htim_motor_1_, htim_motor_2_, htim_motor_3_, htim_motor_4_};
  const uint32_t bitrate = dshot_choose_type(dshot_type) / 20;
  const uint32_t hclk = HAL_RCC_GetHCLKFreq();
  for (unsigned i = 0; i < 4; ++i)
  {
    const bool apb2 = timers[i]->Instance == TIM1 || timers[i]->Instance == TIM8;
    const uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    bool timpre = false;
#if defined(STM32H7)
    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) != 0;
#elif defined(RCC_DCKCFGR1_TIMPRE)
    timpre = (RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) != 0;
#endif
    const uint32_t multiplier = timpre ? 4 : 2;
    const uint32_t clock = pclk * multiplier < hclk ? pclk * multiplier : hclk;
    const dshot_protocol::Timing timing = dshot_protocol::timing(clock, bitrate);
    zero_high_[i] = timing.zero_high;
    one_high_[i] = timing.one_high;
    __HAL_TIM_SET_PRESCALER(timers[i], 0);
    __HAL_TIM_SET_AUTORELOAD(timers[i], timing.period);
    timers[i]->Instance->EGR = TIM_EGR_UG;
  }
}

// __HAL_TIM_DISABLE_DMA is needed to eliminate the delay between different dshot signals
// I don't know why :(
// After adding this function, there are no delay among dshot 1,2,3,4.
void DShot::dshot_dma_tc_callback(DMA_HandleTypeDef* hdma)
{
  TIM_HandleTypeDef* htim = (TIM_HandleTypeDef*)((DMA_HandleTypeDef*)hdma)->Parent;

  if (hdma == htim->hdma[TIM_DMA_ID_CC1])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC2])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC3])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
  }
  else if (hdma == htim->hdma[TIM_DMA_ID_CC4])
  {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
  }
}

void DShot::dshot_put_tc_callback_function()
{
  // TIM_DMA_ID_CCx depends on timer channel
  htim_motor_1_->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_2_->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_3_->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
  htim_motor_4_->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
}

void DShot::dshot_start_pwm()
{
  // Start the timer channel now.
  // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.
  HAL_TIM_PWM_Start(htim_motor_1_, channel_motor_1_);
  HAL_TIM_PWM_Start(htim_motor_2_, channel_motor_2_);
  HAL_TIM_PWM_Start(htim_motor_3_, channel_motor_3_);
  HAL_TIM_PWM_Start(htim_motor_4_, channel_motor_4_);
}

uint16_t DShot::dshot_prepare_packet(uint16_t value, bool dshot_telemetry)
{
  return dshot_protocol::packet(value, dshot_telemetry);
}

// Convert 16 bits packet to 16 pwm signal
void DShot::dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry, unsigned motor)
{
  uint16_t packet;
  packet = dshot_prepare_packet(value, is_telemetry);

  for (int i = 0; i < 16; i++)
  {
    motor_dmabuffer[i] = (packet & 0x8000) ? one_high_[motor] : zero_high_[motor];
    packet <<= 1;
  }

  motor_dmabuffer[16] = 0;
  motor_dmabuffer[17] = 0;
}

void DShot::dshot_prepare_dmabuffer_all(uint16_t* motor_value, bool* is_telemetry)
{
  dshot_prepare_dmabuffer(motor1_dmabuffer_, motor_value[0], is_telemetry[0], 0);
  dshot_prepare_dmabuffer(motor2_dmabuffer_, motor_value[1], is_telemetry[1], 1);
  dshot_prepare_dmabuffer(motor3_dmabuffer_, motor_value[2], is_telemetry[2], 2);
  dshot_prepare_dmabuffer(motor4_dmabuffer_, motor_value[3], is_telemetry[3], 3);
}

bool DShot::dshot_dma_start()
{
  DMA_HandleTypeDef* dma[4] = {htim_motor_1_->hdma[TIM_DMA_ID_CC1], htim_motor_2_->hdma[TIM_DMA_ID_CC2],
    htim_motor_3_->hdma[TIM_DMA_ID_CC3], htim_motor_4_->hdma[TIM_DMA_ID_CC4]};
  uint32_t* buffers[4] = {motor1_dmabuffer_, motor2_dmabuffer_, motor3_dmabuffer_, motor4_dmabuffer_};
  volatile uint32_t* ccr[4] = {&htim_motor_1_->Instance->CCR1, &htim_motor_2_->Instance->CCR2,
    &htim_motor_3_->Instance->CCR3, &htim_motor_4_->Instance->CCR4};
  for (unsigned i = 0; i < 4; ++i)
  {
    if (HAL_DMA_Start_IT(dma[i], (uint32_t)buffers[i], (uint32_t)ccr[i], DSHOT_DMA_BUFFER_SIZE) != HAL_OK)
    {
      for (unsigned j = 0; j < i; ++j) HAL_DMA_Abort(dma[j]);
      return false;
    }
  }
  return true;
}

void DShot::dshot_enable_dma_request()
{
  __HAL_TIM_ENABLE_DMA(htim_motor_1_, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(htim_motor_2_, TIM_DMA_CC2);
  __HAL_TIM_ENABLE_DMA(htim_motor_3_, TIM_DMA_CC3);
  __HAL_TIM_ENABLE_DMA(htim_motor_4_, TIM_DMA_CC4);
}

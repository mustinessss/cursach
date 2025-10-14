// Минимальная версия генератора сигналов STM32F446RE
#define DAC_PIN PA4
#define LED_PIN PA5

void setup() {
  Serial.begin(115200);
  pinMode(DAC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(12);
  
  // Настройка аппаратного таймера для DAC
  setupDACTimer();
  
  Serial.println("STM32 DAC Generator STARTED");
}

void setupDACTimer() {
  // Включаем тактирование TIM2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  
  // Настройка TIM2 для 10kHz
  TIM2->PSC = 8399;    // Prescaler
  TIM2->ARR = 199;     // Auto-reload (84MHz / (8400*200) = 10kHz)
  TIM2->CR1 = TIM_CR1_CEN; // Включить таймер
  
  // Включаем прерывание по обновлению
  TIM2->DIER |= TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler() {
  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF; // Сброс флага
    
    static uint32_t phase = 0;
    phase += 1000; // Изменение фазы
    
    // Простая синусоида
    uint16_t value = 2048 + (uint16_t)(sin(phase * 0.001) * 1000);
    analogWrite(DAC_PIN, value);
    
    // Отправка данных каждые 100 семплов
    static uint16_t sampleCount = 0;
    if (sampleCount++ >= 100) {
      sampleCount = 0;
      Serial.print("DAC:");
      Serial.println(value);
    }
  }
}

void loop() {
  // Мигание светодиода
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }
  
  delay(10);
}
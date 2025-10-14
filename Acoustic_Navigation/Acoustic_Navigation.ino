// =============================================
// АКУСТИЧЕСКАЯ НАВИГАЦИОННАЯ СИСТЕМА
// STM32F446RE + 3 динамика + 1 микрофон
// =============================================

// === КОНФИГУРАЦИЯ ===
#define NUM_SPEAKERS 3
#define BARKER_LENGTH 7
#define SAMPLE_RATE 48000
#define SOUND_SPEED 343.0 // скорость звука, м/с

// === ПИНЫ ===
#define MIC_PIN PA0      // АЦП микрофона
#define DAC_PIN PA4      // ЦАП для динамиков  
#define LED_PIN PA5      // Индикация работы
#define SPK1_PIN PC1     // Управление динамиком 1
#define SPK2_PIN PC2     // Управление динамиком 2
#define SPK3_PIN PC3     // Управление динамиком 3

// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
// Код Баркера длиной 7 (оптимальный)
const int16_t barker7[BARKER_LENGTH] = {1, 1, 1, -1, -1, 1, -1};

// Расстояния до динамиков (в метрах)
float distances[NUM_SPEAKERS] = {0.5, 0.7, 0.9}; // тестовые значения

// Координаты микрофона (x, y в мм)
float coordinates[2] = {120, 180}; // тестовые значения

// Координаты динамиков (в мм)
const float speakerPositions[NUM_SPEAKERS][2] = {
  {0, 250},   // Динамик 1: x=0, y=250mm
  {250, 250}, // Динамик 2: x=250, y=250mm  
  {250, 0}    // Динамик 3: x=250, y=0mm
};

// === НАСТРОЙКА ===
void setup() {
  // Инициализация Serial (USB)
  Serial.begin(115200);
  
  // Настройка пинов
  setupPins();
  
  // Пауза для стабилизации
  delay(2000);
  
  Serial.println("=======================================");
  Serial.println("АКУСТИЧЕСКАЯ НАВИГАЦИОННАЯ СИСТЕМА");
  Serial.println("STM32F446RE + 3 динамика + 1 микрофон");
  Serial.println("=======================================");
  Serial.println("Система готова к работе");
  Serial.println();
}

// === ОСНОВНОЙ ЦИКЛ ===
void loop() {
  // 1. Измеряем расстояния до всех динамиков
  measureAllDistances();
  
  // 2. Вычисляем координаты микрофона
  calculateCoordinates();
  
  // 3. Отправляем данные на компьютер
  sendDataToPC();
  
  // 4. Индикация работы
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  
  // 5. Задержка между измерениями (10 раз в секунду)
  delay(100);
}

// === НАСТРОЙКА ПИНОВ ===
void setupPins() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SPK1_PIN, OUTPUT);
  pinMode(SPK2_PIN, OUTPUT);
  pinMode(SPK3_PIN, OUTPUT);
  
  // Настройка разрешения ЦАП
  analogWriteResolution(12);
  
  // Изначально все динамики выключены
  digitalWrite(SPK1_PIN, LOW);
  digitalWrite(SPK2_PIN, LOW);
  digitalWrite(SPK3_PIN, LOW);
  
  // Включаем светодиод
  digitalWrite(LED_PIN, HIGH);
}

// === ИЗМЕРЕНИЕ РАССТОЯНИЙ ===
void measureAllDistances() {
  Serial.println("--- Измерение расстояний ---");
  
  for(int i = 0; i < NUM_SPEAKERS; i++) {
    Serial.print("Динамик ");
    Serial.print(i + 1);
    Serial.print(": ");
    
    // Активируем динамик
    activateSpeaker(i);
    
    // Здесь будет измерение расстояния
    // Пока используем тестовые значения + небольшой шум
    distances[i] = 0.5 + (i * 0.2) + (random(0, 10) / 100.0);
    
    Serial.print(distances[i], 3);
    Serial.println(" м");
    
    // Выключаем динамик
    deactivateAllSpeakers();
    
    delay(50); // Пауза между измерениями
  }
  Serial.println();
}

// === АКТИВАЦИЯ ДИНАМИКА ===
void activateSpeaker(int speakerNum) {
  // Выключаем все динамики
  deactivateAllSpeakers();
  
  // Включаем нужный динамик
  switch(speakerNum) {
    case 0: 
      digitalWrite(SPK1_PIN, HIGH);
      break;
    case 1:
      digitalWrite(SPK2_PIN, HIGH); 
      break;
    case 2:
      digitalWrite(SPK3_PIN, HIGH);
      break;
  }
}

// === ВЫКЛЮЧЕНИЕ ВСЕХ ДИНАМИКОВ ===
void deactivateAllSpeakers() {
  digitalWrite(SPK1_PIN, LOW);
  digitalWrite(SPK2_PIN, LOW);
  digitalWrite(SPK3_PIN, LOW);
}

// === ВЫЧИСЛЕНИЕ КООРДИНАТ ===
void calculateCoordinates() {
  // Пока используем упрощенный расчет для теста
  // В реальной системе здесь будет трилатерация
  
  // Просто добавляем небольшое движение к тестовым координатам
  coordinates[0] = 120 + (random(0, 20) - 10); // x ± 10mm
  coordinates[1] = 180 + (random(0, 20) - 10); // y ± 10mm
  
  Serial.println("--- Вычисление координат ---");
  Serial.print("Координаты микрофона: ");
  Serial.print("X=");
  Serial.print(coordinates[0], 1);
  Serial.print("mm, Y="); 
  Serial.print(coordinates[1], 1);
  Serial.println("mm");
  Serial.println();
}

// === ПЕРЕДАЧА ДАННЫХ НА КОМПЬЮТЕР ===
void sendDataToPC() {
  // Формат для Python программы:
  
  // Координаты
  Serial.print("COORD:");
  Serial.print(coordinates[0], 2);
  Serial.print(",");
  Serial.print(coordinates[1], 2);
  Serial.println();
  
  // Расстояния
  Serial.print("DIST:");
  for(int i = 0; i < NUM_SPEAKERS; i++) {
    Serial.print(distances[i], 3);
    if(i < NUM_SPEAKERS - 1) Serial.print(",");
  }
  Serial.println();
  
  // Позиции динамиков (для отладки)
  Serial.print("SPK_POS:");
  for(int i = 0; i < NUM_SPEAKERS; i++) {
    Serial.print(speakerPositions[i][0], 0);
    Serial.print(",");
    Serial.print(speakerPositions[i][1], 0);
    if(i < NUM_SPEAKERS - 1) Serial.print(";");
  }
  Serial.println();
  
  Serial.println("---"); // разделитель пакетов
}
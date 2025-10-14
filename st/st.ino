// Простейший тест DAC
void setup() {
  Serial.begin(115200);
  pinMode(PA4, OUTPUT);
  analogWriteResolution(12);
  Serial.println("STM32 DAC TEST");
}

void loop() {
  static uint16_t val = 0;
  analogWrite(PA4, val);
  Serial.println(val);
  val = (val + 100) % 4096;
  delay(10);
}
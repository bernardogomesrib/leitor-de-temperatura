#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;
sensors_event_t umidade, temp;
// Definição dos pinos analógicos
const int PINO_A0 = A0;
const int PINO_A1 = A1;
const int POS_PIN_0 = 4;
const int POS_PIN_1 = 6;
const int AQUECEDOR = 5;
int HISTERESE = 2;
int TEMP_DESEJADA = 65;
bool FLIP = true;
float HUMIDADE_DE_ATIVACAO=15.0;
// Parâmetros do Circuito e do NTC
const float R_REF = 9950.0;       // Resistor fixo de 10k ohms
const float NTC_NOMINAL = 9990.0;  // Resistência do NTC a 25 °C (mude para 8000.0 se o seu for de 8k)
const float TEMP_NOMINAL = 25.0;   // Temperatura nominal (25 °C)
const float BETA = 3950.0;         // Coeficiente Beta padrão
const float TENSAO_REF = 4.8;      // Alimentação de 5V do Arduino

// Função para ler o pino e tirar uma média do ADC
float lerADC(int pino) {
  long soma = 0;
  long valor =0;
  if(pino == PINO_A1){
    digitalWrite(POS_PIN_1,HIGH);
  }else{
    digitalWrite(POS_PIN_0,HIGH);
  }
  delay(1);
  for (int i = 0; i < 2; i++) {
    valor = analogRead(pino);

    //Serial.println(valor);
    soma += valor;

  }
  digitalWrite(POS_PIN_0,LOW);
  digitalWrite(POS_PIN_1,LOW);
  return (float)soma / 2;
}

// Função para calcular a temperatura recebendo a leitura ADC do pino
float calcularTemperatura(float valorADC) {
  if (valorADC >= 1023.0) valorADC = 1022.0;
  if (valorADC <= 0.0) valorADC = 1.0;

  // FÓRMULA CORRIGIDA para NTC no VCC e R_REF no GND:
  float resistencia = R_REF * ((1023.0 / valorADC) - 1.0);

  // 2. Equação de Beta para temperatura em Kelvin
  float tempKelvin = 1.0 / ((1.0 / (TEMP_NOMINAL + 273.15)) + (1.0 / BETA) * log(resistencia / NTC_NOMINAL));

  // 3. Converte para Celsius
  return tempKelvin - 273.15;
}

void setup() {
  pinMode(POS_PIN_0, OUTPUT);
  pinMode(POS_PIN_1, OUTPUT);
  pinMode(AQUECEDOR, OUTPUT);
  Serial.begin(9600);

  // Tenta inicializar no endereço padrão (0x38)
  if (!aht.begin()) { 
    Serial.println("Sensor AHT10 não encontrado!");
    while (1) delay(10);
  }
  Serial.println("AHT10 encontrado com sucesso!");
}

void loop() {
  // --- LEITURA DO PINO A0 ---
  float adcA0 = lerADC(PINO_A0);
  //Serial.println(adcA0);
  float tensaoA0 = (adcA0 * TENSAO_REF) / 1023.0;
  float tempA0 = calcularTemperatura(adcA0);

  // --- LEITURA DO PINO A1 ---
  float adcA1 = lerADC(PINO_A1);
  //Serial.println(adcA1);
  float tensaoA1 = (adcA1 * TENSAO_REF) / 1023.0;
  float tempA1 = calcularTemperatura(adcA1);

  // Exibição A0
  Serial.print("A0 -> ");
  Serial.print(tensaoA0, 2);
  Serial.print("V | ");
  Serial.print(tempA0, 1);
  Serial.print(" °C | ");

  // Exibição A1
  Serial.print("A1 -> ");
  Serial.print(tensaoA1, 2);
  Serial.print("V | ");
  Serial.print(tempA1, 1);
  Serial.print(" °C ");


  // aht10
  //temperatura
  aht.getEvent(&umidade, &temp);
  Serial.print("| AHT10: ");
  Serial.print(temp.temperature, 2); // 2 casas decimais
  Serial.print(" °C");

  Serial.print("  |  ");

  // Exibe a Umidade Relativa
  Serial.print("Umidade: ");
  Serial.print(umidade.relative_humidity, 2); // 2 casas decimais
  Serial.print(" % ");

/**
  FLIP = !FLIP;
  digitalWrite(AQUECEDOR,FLIP?HIGH:LOW);
*/
  if (tempA1 < (TEMP_DESEJADA - HISTERESE) || umidade.relative_humidity >HUMIDADE_DE_ATIVACAO) {
    FLIP= true;
    digitalWrite(AQUECEDOR, HIGH); // Liga o aquecedor/SSR
  } 
  else if (tempA1 > (TEMP_DESEJADA + HISTERESE)) {
    FLIP = false;
    digitalWrite(AQUECEDOR, LOW);  // Desliga o aquecedor/SSR
  }
  Serial.print("Aquecedor está:");
  Serial.println(FLIP?"LIGADO ":"DESLIGADO ");


  delay(2000);
}
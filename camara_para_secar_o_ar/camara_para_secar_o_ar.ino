#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>

// Definições de largura e altura do display (em pixels)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Reset do display (não usado na maioria dos módulos de 4 pinos, usamos -1)
#define OLED_RESET -1
// Cria o objeto para controlar a tela
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//definição do AHT10
Adafruit_AHTX0 aht;
sensors_event_t umidade, temp;

/* bool FLAG_ATUALIZAR_TELA=false;
void bota pra atualizar */


// Código de validação (Magic Number) para verificar se a EEPROM já foi gravada alguma vez
const uint32_t MAGIC_NUMBER = 0xA5B6C7D8;
// Definição dos pinos analógicos e pinos do aquecedor e de ativação dos NTC's
const uint8_t PINO_A0 = A0;
const uint8_t PINO_A1 = A1;
const uint8_t POS_PIN_0 = 4;
const uint8_t POS_PIN_1 = 6;
const uint8_t AQUECEDOR = 5;

// --- PINAGEM DA PLACA DE BOTÕES ---
const uint8_t PIN_ENC_A = 40;   // Lado esquerdo do pino central
const uint8_t PIN_ENC_B = 38;   // Lado direito do pino central
const uint8_t PIN_SW_ENC = 36;  // Botão do encoder
const uint8_t PIN_BT1 = 34;     // Botão adicional 1
const uint8_t PIN_BT2 = 32;     // Botão adicional 2

//variaveis
enum TelaEstado : uint8_t {
  TELA_INICIAL,                //TELA INCIAL, SÓ MOSTRA TODOS OS VALORES
  TELA_TEMPORIZADOR,           //TELA ANTES DE INICIAR O TEMPORIZADOR
  TELA_TEMPORIZADOR_CONTANDO,  //TELA CONTANDO O TEMPORIZADOR CONFIGURADO ANTES
  TELA_CONFIGURACAO,           //CONFIGURANDO HISTERESE, HUMIDADE DE ATIVAÇÃO MINIMA, TEMPERATURA DE AQUECIMENTO
  TELA_AJUSTE_TEMP,            //TELA PARA AJUSTAR PARÂMETROS DO CIRCUITO DE NTC PARA O CALCULO DE TEMPERATURA COM NTC
};

// =========================================================================
// ESTRUTURA DE LISTA DINÂMICA COMPATÍVEL COM AVR (Substitui o std::vector)
// =========================================================================
struct ListaErros {
  String* itens = nullptr;
  size_t capacidade = 0;
  size_t total = 0;

  ~ListaErros() {
    delete[] itens;
  }

  void push_back(const String& msg) {
    if (total >= capacidade) {
      size_t novaCapacidade = (capacidade == 0) ? 2 : capacidade * 2;
      String* novoArray = new String[novaCapacidade];
      for (size_t i = 0; i < total; i++) {
        novoArray[i] = itens[i];
      }
      delete[] itens;
      itens = novoArray;
      capacidade = novaCapacidade;
    }
    itens[total++] = msg;
  }

  size_t size() const {
    return total;
  }
  bool empty() const {
    return total == 0;
  }
  void clear() {
    total = 0;
  }

  String& operator[](size_t index) {
    return itens[index];
  }
  const String& operator[](size_t index) const {
    return itens[index];
  }
};

ListaErros listaErros;
// Variável que guarda a tela ativa no momento
TelaEstado telaAtual = TELA_INICIAL;

// Endereço inicial da EEPROM (bloco 0)
const int ENDERECO_EEPROM = 0;

struct ConfigSistema {
  float R_REF;                    // Resistor fixo de 10k ohms - deve ser salvo
  float NTC_NOMINAL;              // Resistência do NTC a 25 °C (mude para 8000.0 se o seu for de 8k) - deve ser salvo
  float TEMP_NOMINAL;             // Temperatura nominal (25 °C) - deve ser salvo
  float BETA;                     // Coeficiente Beta padrão - deve ser salvo
  float TENSAO_REF;               // Alimentação de 5V do Arduino - deve ser salvo
  uint8_t HISTERESE;              //deve ser salvo
  float TEMP_DESEJADA;            //deve ser salvo
  float HUMIDADE_DE_ATIVACAO;     //deve ser salvo
  uint8_t TEMPORIZADOR_HORAS;     //deve ser salvo
  uint8_t TEMPORIZADOR_MINUTOS;   //deve ser salvo
  uint8_t TEMPORIZADOR_SEGUNDOS;  //deve ser salvo
  bool IS_CELCIUS;                // definindo se é celcius ou farenheight
  uint32_t DEBOUNCER_TIME;        //tempo de debounce
  uint32_t assinatura;            // 4 bytes para validação
};


ConfigSistema cfgvar = ConfigSistema();

void carregarConfiguracoes() {
  // LÊ A STRUCT INTEIRA DA EEPROM EM 1 ÚNICA CHAMADA
  EEPROM.get(ENDERECO_EEPROM, cfgvar);

  // Se a EEPROM for nova/virgem, carrega os padrões e salva
  if (cfgvar.assinatura != MAGIC_NUMBER) {
    carregarValoresPadrao();
    //por enquanto está sem gravar nada, quero que se foda, por enuanto só a existência disso tá bom, vou mudar muita coisa.
    //salvarConfiguracoes(); // Grava a struct inteira inicial na EEPROM
  }
}

void salvarConfiguracoes() {
  // SALVA A STRUCT INTEIRA NA EEPROM EM 1 ÚNICA CHAMADA
  // O .put() só regrava os bytes que realmente mudaram, protegendo a vida útil da EEPROM!
  EEPROM.put(ENDERECO_EEPROM, cfgvar);
}

void carregarValoresPadrao() {
  cfgvar.R_REF = 9950.0;
  cfgvar.NTC_NOMINAL = 10000.0;
  cfgvar.TEMP_NOMINAL = 25.0;
  cfgvar.BETA = 3950.0;
  cfgvar.TENSAO_REF = 5.0;
  cfgvar.HISTERESE = 2;
  cfgvar.TEMP_DESEJADA = 65;
  cfgvar.HUMIDADE_DE_ATIVACAO = 15.0;
  cfgvar.TEMPORIZADOR_HORAS = 2;
  cfgvar.TEMPORIZADOR_MINUTOS = 0;
  cfgvar.TEMPORIZADOR_SEGUNDOS = 0;
  cfgvar.IS_CELCIUS = true;
  cfgvar.DEBOUNCER_TIME = 25;
  cfgvar.assinatura = MAGIC_NUMBER;  // Marca a EEPROM como inicializada
}
//variaveis do ambiente
bool AQUECEDOR_ATIVADO = false;  // var de status
float TEMP_AMBIENTE = 0;         //var de leitura
float TEMP_AQUECEDOR = 0;        //var de leitura
float TEMP_AH10 = 0;             //var de leitura
float HUMIDADE_ATUAL = 0;        //var de leitura
//variaveis de funções
bool PAUSE = false;  //var de pausar o tempo no menu de contagem lá
//var de tempo para contar o tempo com base nas configs e ser editado também para salvar!
uint8_t HR = 0;
uint8_t MIN = 0;
uint8_t SEG = 0;
uint8_t MENU_ATUAL;  //Var para definir qual o menu atualmente está selecionado ou variavel, será alterada sempre com base na var ENCODER_VALUE e com base na tela atual.

// Variaveis para botões e encoder
uint8_t ENCODER_VALUE = 0;
uint8_t BOTAO_ENCODER = 0;
uint8_t BOTAO_1 = 0;
uint8_t BOTAO_2 = 0;


// Função para ler o pino e tirar uma média do ADC
float lerADC(int pino) {
  long soma = 0;
  long valor = 0;
  if (pino == PINO_A1) {
    digitalWrite(POS_PIN_1, HIGH);
  } else {
    digitalWrite(POS_PIN_0, HIGH);
  }
  delay(1);
  for (int i = 0; i < 2; i++) {
    valor = analogRead(pino);

    //Serial.println(valor);
    soma += valor;
  }
  digitalWrite(POS_PIN_0, LOW);
  digitalWrite(POS_PIN_1, LOW);
  return (float)soma / 2;
}

// Função para calcular a temperatura recebendo a leitura ADC do pino
float calcularTemperatura(float valorADC) {
  if (valorADC >= 1023.0) valorADC = 1022.0;
  if (valorADC <= 0.0) valorADC = 1.0;

  // FÓRMULA CORRIGIDA para NTC no VCC e R_REF no GND:
  float resistencia = cfgvar.R_REF * ((1023.0 / valorADC) - 1.0);

  // 2. Equação de Beta para temperatura em Kelvin
  float tempKelvin = 1.0 / ((1.0 / (cfgvar.TEMP_NOMINAL + 273.15)) + (1.0 / cfgvar.BETA) * log(resistencia / cfgvar.NTC_NOMINAL));

  // 3. Converte para Celsius
  if (cfgvar.IS_CELCIUS)
    return tempKelvin - 273.15;
  else
    return tempKelvin - 273.15 * 1.8 + 32;
}

void mostrarTudo() {
  display.clearDisplay();

  // Configurações do texto
  display.setTextSize(1);               // Tamanho da fonte (1 é o padrão)
  display.setTextColor(SSD1306_WHITE);  // Cor do texto
  display.setCursor(0, 0);              // Posição inicial (X=0, Y=0)

  // Escrevendo na tela
  switch (telaAtual) {
    case TELA_INICIAL:
      display.print("Temp. Ambiente: ");
      display.println(TEMP_AMBIENTE);
      display.print("Temp. Aquecedor: ");
      display.println(TEMP_AQUECEDOR);
      display.print("Temp. AH10: ");
      display.println(TEMP_AH10);
      display.print("Humidade: ");
      display.println(HUMIDADE_ATUAL);
      display.print("Aquecedor: ");
      display.println(AQUECEDOR_ATIVADO ? "Ativo" : "desativado");

      for (int i = 0; i < listaErros.size() || i < 2; i++) {
        display.println(listaErros[i]);
      }
      display.println("B1:Temp B2: Configs");
      if (BOTAO_1 > 0) {
        telaAtual = TELA_TEMPORIZADOR;
        BOTAO_1 = 0;
      }
      if (BOTAO_2 > 0) {
        telaAtual = TELA_CONFIGURACAO;
        BOTAO_2 = 0;
      }
      break;
    case TELA_TEMPORIZADOR:
      display.setTextSize(2);
      display.println("Tempo ativado");
      display.setTextSize(1);
      //quadrado com uma cópia dos valores atuais de   cfgvar.TEMPORIZADOR_HORAS=2;
      //cfgvar.TEMPORIZADOR_MINUTOS=0;
      //cfgvar.TEMPORIZADOR_SEGUNDOS=0;
      // para configurar o tempo e o que estiver atualmente "piscando" deve ser alterado com a roda do encoder, somando e subtraindo o
      //valor de encoder e subtraindo para fazer com que ele chegue a 0.
      display.println("B1:Voltar B2:Contar BE:mudar");  // ULTIMA LINHA TEM QUE SER ISSO, não sei se vai ficar grande de mais, tenho que pensar em como fazer o carrocel
      if (BOTAO_1 > 0) {
        telaAtual = TELA_INICIAL;
        BOTAO_1 = 0;
      }
      if (BOTAO_2 > 0) {
        telaAtual = TELA_TEMPORIZADOR_CONTANDO;
        BOTAO_2 = 0;
      }
      break;
    case TELA_TEMPORIZADOR_CONTANDO:

      //colocar logica também de iniciar o aquecedor ou quando pausar desligar o aquecedor
      display.setTextSize(2);
      display.println("Contando...");
      //quadrado com o tempo contando

      display.print("tmp.amb: ");
      display.print(TEMP_AMBIENTE, 2);
      display.print(" temp.aq: ");
      display.println(TEMP_AQUECEDOR, 2);
      display.print(" Hum:");
      display.println(HUMIDADE_ATUAL, 2);
      //ultima linha
      display.print("B1:Cancelar B2:Pausar");
      if (BOTAO_1 > 1) {
        AQUECEDOR_ATIVADO = false;
        telaAtual = TELA_INICIAL;
        BOTAO_1 = 0;
      }
      if (BOTAO_2 > 1) {
        AQUECEDOR_ATIVADO = false;
        PAUSE = true;
        BOTAO_2 = 0;
      }


      break;
    case TELA_CONFIGURACAO:
      display.setTextSize(2);
      display.println("tela config");
      display.setTextSize(1);
      if (BOTAO_1 > 0) {
        telaAtual = TELA_INICIAL;
        BOTAO_1 = 0;
      }
  }

  // Envia as instruções de desenho para a tela física
  display.display();
}

// =========================================================================
// 2. INTERRUPÇÃO DO ENCODER ROTATIVO (Pinos 40 e 38)
// =========================================================================
volatile uint32_t ultimoTempoInterrupcaoBT = 0;

void isrEncoder() {

  uint8_t estadoA = digitalRead(PIN_ENC_A);
  uint8_t estadoB = digitalRead(PIN_ENC_B);

  // Detecta o sentido de rotação
  if (estadoA == LOW) {
    if (estadoB == HIGH) {
      ENCODER_VALUE += 1;
    } else {
      ENCODER_VALUE -= 1;
    }
  }
}

void isrBotaoEncoder() {
  uint32_t agora = millis();
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;  // Debounce
  ultimoTempoInterrupcaoBT = agora;

  if (digitalRead(PIN_SW_ENC) == LOW) {
    BOTAO_ENCODER += 1;
  }
}

void isrBotao1() {
  uint32_t agora = millis();
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;  // Debounce
  ultimoTempoInterrupcaoBT = agora;

  if (digitalRead(PIN_BT1) == LOW) {
    BOTAO_1 += 1;
  }
}
void isrBotao2() {
  uint32_t agora = millis();
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;  // Debounce
  ultimoTempoInterrupcaoBT = agora;

  if (digitalRead(PIN_BT2) == LOW) {
    BOTAO_2 += 1;
  }
}
void setup() {
  pinMode(POS_PIN_0, OUTPUT);
  pinMode(POS_PIN_1, OUTPUT);
  pinMode(AQUECEDOR, OUTPUT);
  // 1. Configuração dos Pinos como INPUT_PULLUP
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_SW_ENC, INPUT_PULLUP);
  pinMode(PIN_BT1, INPUT_PULLUP);
  pinMode(PIN_BT2, INPUT_PULLUP);
  Serial.begin(9600);

  //lendo as variaveis da EEPROM:



  // Tenta inicializar no endereço padrão (0x38)
  if (!aht.begin()) {
    Serial.println("Sensor AHT10 não encontrado!");
    while (1) delay(10);
  }
  Serial.println("AHT10 encontrado com sucesso!");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao inicializar o OLED SSD1306"));
  }


  // Anexa Interrupções por Mudança de Pino (PinChangeInterrupt)
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ENC_A), isrEncoder, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SW_ENC), isrBotaoEncoder, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BT1), isrBotao1, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BT2), isrBotao2, FALLING);
  // Limpa o buffer da memória da tela
  mostrarTudo();
}
unsigned long ultimaAtualizacaoTela = 0;
unsigned long ultimaLeituraSensores = 0;

const unsigned long INTERVALO_TELA = 80;
const unsigned long INTERVALO_SENSORES = 2000;
void loop() {
  unsigned long tempoAtual = millis();
  // --- LEITURA DO PINO A0 ---
  if (tempoAtual - ultimaLeituraSensores >= INTERVALO_SENSORES) {
    ultimaLeituraSensores = tempoAtual;
    float adcA0 = lerADC(PINO_A0);
    //Serial.println(adcA0);
    float tensaoA0 = (adcA0 * cfgvar.TENSAO_REF) / 1023.0;
    float tempA0 = calcularTemperatura(adcA0);

    // --- LEITURA DO PINO A1 ---
    float adcA1 = lerADC(PINO_A1);
    //Serial.println(adcA1);
    float tensaoA1 = (adcA1 * cfgvar.TENSAO_REF) / 1023.0;
    float tempA1 = calcularTemperatura(adcA1);

    // Exibição A0
    Serial.print("A0 -> ");
    Serial.print(tensaoA0, 2);
    Serial.print("V | ");
    Serial.print(tempA0, 1);
    TEMP_AMBIENTE = tempA0;
    Serial.print(" °C | ");

    // Exibição A1
    Serial.print("A1 -> ");
    Serial.print(tensaoA1, 2);
    Serial.print("V | ");
    Serial.print(tempA1, 1);
    TEMP_AQUECEDOR = tempA1;
    Serial.print(" °C ");


    // aht10
    //temperatura
    aht.getEvent(&umidade, &temp);
    Serial.print("| AHT10: ");
    Serial.print(temp.temperature, 2);
    TEMP_AH10 = temp.temperature;
    Serial.print(" °C");

    Serial.print("  |  ");

    // Exibe a Umidade Relativa
    Serial.print("Umidade: ");
    Serial.print(umidade.relative_humidity, 2);
    HUMIDADE_ATUAL = umidade.relative_humidity;
    Serial.print(" % ");

    /**
    Serial.print("Aquecedor está:");
    Serial.println(AQUECEDOR_ATIVADO ? "LIGADO " : "DESLIGADO ");
  AQUECEDOR_ATIVADO = !AQUECEDOR_ATIVADO;
  digitalWrite(AQUECEDOR,AQUECEDOR_ATIVADO?HIGH:LOW);
*/
    /*   if (tempA1 < (cfgvar.TEMP_DESEJADA - cfgvar.HISTERESE) || umidade.relative_humidity >cfgvar.HUMIDADE_DE_ATIVACAO) {
    AQUECEDOR_ATIVADO= true;
    digitalWrite(AQUECEDOR, HIGH); // Liga o aquecedor/SSR
  } 
  else if (tempA1 > (cfgvar.TEMP_DESEJADA + cfgvar.HISTERESE)) {
    AQUECEDOR_ATIVADO = false;
    digitalWrite(AQUECEDOR, LOW);  // Desliga o aquecedor/SSR
  }
 */
  }
  if (tempoAtual - ultimaAtualizacaoTela >= INTERVALO_TELA) {
    ultimaAtualizacaoTela = tempoAtual;
    mostrarTudo();
  }
}
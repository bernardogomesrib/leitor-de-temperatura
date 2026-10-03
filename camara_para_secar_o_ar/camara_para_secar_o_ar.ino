#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_AHTX0 aht;
sensors_event_t umidade, temp;

const uint32_t MAGIC_NUMBER = 0xA5B6C7D8;

const uint8_t PINO_A0 = A0;
const uint8_t PINO_A1 = A1;
const uint8_t POS_PIN_0 = 4;
const uint8_t POS_PIN_1 = 6;
const uint8_t AQUECEDOR = 5;

// --- PINAGEM DA PLACA DE BOTÕES ---
const uint8_t PIN_ENC_A = 50;
const uint8_t PIN_ENC_B = 52;
const uint8_t PIN_SW_ENC = 53;
const uint8_t PIN_BT1 = A14;
const uint8_t PIN_BT2 = A15;

enum TelaEstado : uint8_t {
  TELA_INICIAL,
  TELA_TEMPORIZADOR,
  TELA_TEMPORIZADOR_CONTANDO,
  TELA_CONFIGURACAO,
  TELA_AJUSTE_TEMP,
};

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
TelaEstado telaAtual = TELA_INICIAL;
const int ENDERECO_EEPROM = 0;

struct ConfigSistema {
  float R_REF;
  float NTC_NOMINAL;
  float TEMP_NOMINAL;
  float BETA;
  float TENSAO_REF;
  uint8_t HISTERESE;
  float TEMP_DESEJADA;
  float HUMIDADE_DE_ATIVACAO;
  uint8_t TEMPORIZADOR_HORAS;
  uint8_t TEMPORIZADOR_MINUTOS;
  uint8_t TEMPORIZADOR_SEGUNDOS;
  uint8_t INTERVALO_REATIVACAO_HORAS;
  bool IS_CELCIUS;
  uint32_t DEBOUNCER_TIME;
  uint32_t assinatura;
};

ConfigSistema cfgvar = ConfigSistema();

void carregarConfiguracoes();
void salvarConfiguracoes();
void carregarValoresPadrao();

// Variaveis de ambiente
bool AQUECEDOR_ATIVADO = false;
float TEMP_AMBIENTE = 0;
float TEMP_AQUECEDOR = 0;
float TEMP_AH10 = 0;
float HUMIDADE_ATUAL = 0;

// Controle do Temporizador
bool PAUSE = false;
uint8_t HR = 0;
uint8_t MIN = 0;
uint8_t SEG = 0;
uint32_t tempoTotalSegundos = 0;
unsigned long ultimoSegundoTimer = 0;

// Controle do ciclo de aquecimento automático
bool aquecimentoAtivo = false;            // Flag global de ciclo em andamento
unsigned long ultimaDesativacaoMs = 0;    // Guarda o timestamp (millis) da última desativação
const unsigned long INTERVALO_5H = 18000000UL; // 5 horas em milissegundos (5 * 3600 * 1000)

// Estado de edição do temporizador (0: HH, 1: MM, 2: SS)
uint8_t focadoTempo = 0;

// Estado do Carrossel do Menu de Configuração
uint8_t itemConfigSelecionado = 0;
bool modoEdicaoConfig = false;
const uint8_t TOTAL_ITENS_CONFIG = 7;

// Variáveis dos botões e encoder (tratados em loop)
volatile int8_t deltaEncoder = 0;
volatile bool flagBotaoEncoder = false;
volatile bool flagBotao1 = false;
volatile bool flagBotao2 = false;

volatile uint32_t ultimoTempoInterrupcaoBT = 0;

// Variáveis de diagnóstico de falha do aquecedor
unsigned long tempoInicioAquecedorMs = 0;
float tempInicialAquecedor = 0.0;
bool monitorandoAquecimento = false;
bool testeInicialConcluido = false;

void gerenciarAquecimento() {
  // 1. Verificação de disparo automático por umidade alta
  if (!aquecimentoAtivo) {
    unsigned long intervaloMs = (unsigned long)cfgvar.INTERVALO_REATIVACAO_HORAS * 3600000UL;
    bool tempoPassado = (ultimaDesativacaoMs == 0) || (millis() - ultimaDesativacaoMs >= intervaloMs);
    
    if (HUMIDADE_ATUAL > cfgvar.HUMIDADE_DE_ATIVACAO && tempoPassado) {
      aquecimentoAtivo = true;
      testeInicialConcluido = false; // Reseta o teste para o novo ciclo
      tempoTotalSegundos = ((uint32_t)cfgvar.TEMPORIZADOR_HORAS * 3600) +
                           ((uint32_t)cfgvar.TEMPORIZADOR_MINUTOS * 60) +
                           cfgvar.TEMPORIZADOR_SEGUNDOS;
      ultimoSegundoTimer = millis();
    }
  }

  // 2. Controle do relé com base na temperatura desejada e histerese
  if (aquecimentoAtivo && !PAUSE) {
    if (TEMP_AQUECEDOR < (cfgvar.TEMP_DESEJADA - cfgvar.HISTERESE)) {
      if (!AQUECEDOR_ATIVADO) {
        AQUECEDOR_ATIVADO = true;
        digitalWrite(AQUECEDOR, HIGH);
        
        // Inicia o monitoramento SOMENTE se o teste do início do ciclo ainda não foi feito
        if (!testeInicialConcluido) {
          tempoInicioAquecedorMs = millis();
          tempInicialAquecedor = TEMP_AQUECEDOR;
          monitorandoAquecimento = true;
        }
      }
    } else if (TEMP_AQUECEDOR > (cfgvar.TEMP_DESEJADA + cfgvar.HISTERESE)) {
      AQUECEDOR_ATIVADO = false;
      digitalWrite(AQUECEDOR, LOW);
      monitorandoAquecimento = false;
    }

    // 3. Validação de Falha: Executada EXCLUSIVAMENTE no primeiro arranque do ciclo
    if (AQUECEDOR_ATIVADO && monitorandoAquecimento && !testeInicialConcluido) {
      if (millis() - tempoInicioAquecedorMs >= 30000UL) { // Passaram 30 segundos
        if ((TEMP_AQUECEDOR - tempInicialAquecedor) < 5.0) {
          // Falha de aquecimento na partida
          AQUECEDOR_ATIVADO = false;
          digitalWrite(AQUECEDOR, LOW);
          aquecimentoAtivo = false;
          monitorandoAquecimento = false;
          testeInicialConcluido = true;
          ultimaDesativacaoMs = millis();

          listaErros.push_back(F("Falha Aquecedor"));
          Serial.println(F("ERRO: Aquecedor nao elevou 5C nos primeiros 30s!"));
        } else {
          // Teste bem-sucedido: marca como concluído para que as religadas de histerese NUNCA mais monitorem
          monitorandoAquecimento = false;
          testeInicialConcluido = true;
          Serial.println(F("OK: Teste inicial do aquecedor aprovado!"));
        }
      }
    }

  } else {
    // Garantia de desligamento caso o ciclo pare ou seja pausado
    AQUECEDOR_ATIVADO = false;
    digitalWrite(AQUECEDOR, LOW);
    monitorandoAquecimento = false;
  }
}

void isrEncoder() {
  uint8_t estadoA = digitalRead(PIN_ENC_A);
  uint8_t estadoB = digitalRead(PIN_ENC_B);
  uint32_t agora = millis();
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;
  ultimoTempoInterrupcaoBT = agora;
  if (estadoA == LOW) {
    if (estadoB == HIGH) {
      deltaEncoder++;
      Serial.println("E+");
    } else {
      deltaEncoder--;
      Serial.println("E-");
    }
  }
}

void isrBotaoEncoder() {
  uint32_t agora = millis();
  Serial.println("BE");
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;
  ultimoTempoInterrupcaoBT = agora;
  if (digitalRead(PIN_SW_ENC) == HIGH) flagBotaoEncoder = true;
}

void isrBotao1() {
  uint32_t agora = millis();
  Serial.println("B1");
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;
  ultimoTempoInterrupcaoBT = agora;
  if (digitalRead(PIN_BT1) == HIGH) flagBotao1 = true;
}

void isrBotao2() {
  uint32_t agora = millis();
  Serial.println("B2");
  if (agora - ultimoTempoInterrupcaoBT < cfgvar.DEBOUNCER_TIME) return;
  ultimoTempoInterrupcaoBT = agora;
  if (digitalRead(PIN_BT2) == HIGH) flagBotao2 = true;
}

float lerADC(int pino) {
  long soma = 0;
  if (pino == PINO_A1) digitalWrite(POS_PIN_1, HIGH);
  else digitalWrite(POS_PIN_0, HIGH);

  delay(1);
  for (int i = 0; i < 2; i++) {
    soma += analogRead(pino);
  }
  digitalWrite(POS_PIN_0, LOW);
  digitalWrite(POS_PIN_1, LOW);
  return (float)soma / 2.0;
}

float calcularTemperatura(float valorADC) {
  if (valorADC >= 1023.0) valorADC = 1022.0;
  if (valorADC <= 0.0) valorADC = 1.0;

  float resistencia = cfgvar.R_REF * ((1023.0 / valorADC) - 1.0);
  float tempKelvin = 1.0 / ((1.0 / (cfgvar.TEMP_NOMINAL + 273.15)) + (1.0 / cfgvar.BETA) * log(resistencia / cfgvar.NTC_NOMINAL));

  if (cfgvar.IS_CELCIUS) return tempKelvin - 273.15;
  else return (tempKelvin - 273.15) * 1.8 + 32.0;
}

void processarInputs() {
  // Captura deltas das ISRs de forma atômica
  noInterrupts();
  int8_t encMove = deltaEncoder;
  deltaEncoder = 0;
  bool btnEnc = flagBotaoEncoder;
  flagBotaoEncoder = false;
  bool btn1 = flagBotao1;
  flagBotao1 = false;
  bool btn2 = flagBotao2;
  flagBotao2 = false;
  interrupts();

  switch (telaAtual) {
    case TELA_INICIAL:
      if (btn1) {
        HR = cfgvar.TEMPORIZADOR_HORAS;
        MIN = cfgvar.TEMPORIZADOR_MINUTOS;
        SEG = cfgvar.TEMPORIZADOR_SEGUNDOS;
        focadoTempo = 0;
        telaAtual = TELA_TEMPORIZADOR;
      } else if (btn2) {
        itemConfigSelecionado = 0;
        modoEdicaoConfig = false;
        telaAtual = TELA_CONFIGURACAO;
      } else if (btnEnc) {
        // Liga/Desliga manual do aquecimento com o tempo padrão das configurações
        if (!aquecimentoAtivo) {
          aquecimentoAtivo = true;
          PAUSE = false;
          testeInicialConcluido = false;
          tempoTotalSegundos = ((uint32_t)cfgvar.TEMPORIZADOR_HORAS * 3600) +
                               ((uint32_t)cfgvar.TEMPORIZADOR_MINUTOS * 60) +
                               cfgvar.TEMPORIZADOR_SEGUNDOS;
          ultimoSegundoTimer = millis();
        } else {
          aquecimentoAtivo = false;
          ultimaDesativacaoMs = millis();
        }
      }
      break;

    case TELA_TEMPORIZADOR:
      if (btn1) {
        telaAtual = TELA_INICIAL;
      } else if (btn2) {
        tempoTotalSegundos = ((uint32_t)HR * 3600) + ((uint32_t)MIN * 60) + SEG;
        if (tempoTotalSegundos > 0) {
          PAUSE = false;
          aquecimentoAtivo = true;
          testeInicialConcluido = false;
          ultimoSegundoTimer = millis();
          telaAtual = TELA_TEMPORIZADOR_CONTANDO;
        }
      } else if (btnEnc) {
        focadoTempo = (focadoTempo + 1) % 3;
      } else if (encMove != 0) {
        if (focadoTempo == 0) {
          int h = HR + encMove;
          HR = (h < 0) ? 23 : ((h > 23) ? 0 : h);
        } else if (focadoTempo == 1) {
          int m = MIN + encMove;
          MIN = (m < 0) ? 59 : ((m > 59) ? 0 : m);
        } else if (focadoTempo == 2) {
          int s = SEG + encMove;
          SEG = (s < 0) ? 59 : ((s > 59) ? 0 : s);
        }
      }
      break;

    case TELA_TEMPORIZADOR_CONTANDO:
      if (btn1) {
        aquecimentoAtivo = false;
        ultimaDesativacaoMs = millis();
        telaAtual = TELA_INICIAL;
      } else if (btn2) {
        PAUSE = !PAUSE;
      }
      break;

   case TELA_CONFIGURACAO:
      if (btn1) {
        if (modoEdicaoConfig) {
          modoEdicaoConfig = false; // Cancela edição do item selecionado
        } else {
          salvarConfiguracoes(); // Salva na EEPROM ao sair por B1
          telaAtual = TELA_INICIAL;
        }
      } else if (btn2) {
        // Recarrega as configurações salvas na EEPROM (descarta alterações não salvas)
        carregarConfiguracoes();
        modoEdicaoConfig = false;
        telaAtual = TELA_INICIAL;
      } else if (btnEnc) {
        if (itemConfigSelecionado == 6) {
          telaAtual = TELA_AJUSTE_TEMP;
          focadoTempo = 0; // Reutilizado na tela de calibração NTC
          modoEdicaoConfig = false;
        } else {
          modoEdicaoConfig = !modoEdicaoConfig;
        }
      } else if (encMove != 0) {
        if (!modoEdicaoConfig) {
          int novoItem = itemConfigSelecionado + encMove;
          if (novoItem < 0) itemConfigSelecionado = TOTAL_ITENS_CONFIG - 1;
          else if (novoItem >= TOTAL_ITENS_CONFIG) itemConfigSelecionado = 0;
          else itemConfigSelecionado = novoItem;
        } else {
          switch (itemConfigSelecionado) {
            case 0:
              cfgvar.TEMP_DESEJADA += encMove * 0.5;
              if (cfgvar.TEMP_DESEJADA < 40.0) cfgvar.TEMP_DESEJADA = 40.0;
              if (cfgvar.TEMP_DESEJADA > 90.0) cfgvar.TEMP_DESEJADA = 90.0;
              break;
            case 1:
              cfgvar.HUMIDADE_DE_ATIVACAO += encMove * 0.5;
              if (cfgvar.HUMIDADE_DE_ATIVACAO < 5.0) cfgvar.HUMIDADE_DE_ATIVACAO = 5.0;
              if (cfgvar.HUMIDADE_DE_ATIVACAO > 90.0) cfgvar.HUMIDADE_DE_ATIVACAO = 90.0;
              break;
            case 2:
              {
                int hist = (int)cfgvar.HISTERESE + encMove;
                if (hist < 1) hist = 1;
                if (hist > 10) hist = 10;
                cfgvar.HISTERESE = hist;
              }
              break;
            case 3: // Novo: Intervalo de Reativação (1h a 24h)
              {
                int intHoras = (int)cfgvar.INTERVALO_REATIVACAO_HORAS + encMove;
                if (intHoras < 1) intHoras = 1;
                if (intHoras > 24) intHoras = 24;
                cfgvar.INTERVALO_REATIVACAO_HORAS = intHoras;
              }
              break;
            case 4:
              cfgvar.IS_CELCIUS = !cfgvar.IS_CELCIUS;
              break;
            case 5:
              {
                int32_t novoDebounce = (int32_t)cfgvar.DEBOUNCER_TIME + (encMove * 5);
                if (novoDebounce < 5) novoDebounce = 5;
                if (novoDebounce > 250) novoDebounce = 250;
                cfgvar.DEBOUNCER_TIME = novoDebounce;
              }
              break;
          }
        }
      }
      break;

    case TELA_AJUSTE_TEMP:
      if (btn1) {
        telaAtual = TELA_CONFIGURACAO;  // Voltar sem alterar modo
      } else if (btnEnc) {
        // Alterna o parâmetro do NTC focado: 0 -> R_REF, 1 -> BETA, 2 -> NTC_NOMINAL
        focadoTempo = (focadoTempo + 1) % 3;
      } else if (encMove != 0) {
        if (focadoTempo == 0) {  // R_REF (Ajuste de 10 em 10 Ohms)
          cfgvar.R_REF += encMove * 10.0;
          if (cfgvar.R_REF < 1000.0) cfgvar.R_REF = 1000.0;
          if (cfgvar.R_REF > 100000.0) cfgvar.R_REF = 100000.0;
        } else if (focadoTempo == 1) {  // BETA (Ajuste de 10 em 10)
          cfgvar.BETA += encMove * 10.0;
          if (cfgvar.BETA < 2000.0) cfgvar.BETA = 2000.0;
          if (cfgvar.BETA > 6000.0) cfgvar.BETA = 6000.0;
        } else if (focadoTempo == 2) {  // NTC25 (Ajuste de 100 em 100 Ohms)
          cfgvar.NTC_NOMINAL += encMove * 100.0;
          if (cfgvar.NTC_NOMINAL < 1000.0) cfgvar.NTC_NOMINAL = 1000.0;
          if (cfgvar.NTC_NOMINAL > 100000.0) cfgvar.NTC_NOMINAL = 100000.0;
        }
      }
      break;
  }
}

void mostrarTudo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  bool piscar = (millis() / 300) % 2 == 0;  // Estado de pisca a ~3.3 Hz

  switch (telaAtual) {
    case TELA_INICIAL:
      display.println(F("--- MONIT. CAMARA ---"));
      display.print(F("Amb: ")); display.print(TEMP_AMBIENTE, 1);display.print("|");display.print(TEMP_AH10, 1); display.println(F("C"));
      display.print(F("Aq: ")); display.print(TEMP_AQUECEDOR, 1); display.print(F("C"));display.print("| ");
      display.print(F("Um: ")); display.print(HUMIDADE_ATUAL, 1); display.println(F("%"));
      
      display.print(F("Ciclo Aq : ")); 
      if (aquecimentoAtivo) {
        uint32_t h = tempoTotalSegundos / 3600;
        uint32_t m = (tempoTotalSegundos % 3600) / 60;
        uint32_t s = tempoTotalSegundos % 60;
        if (h < 10) display.print(F("0")); display.print(h); display.print(F(":"));
        if (m < 10) display.print(F("0")); display.print(m); display.print(F(":"));
        if (s < 10) display.print(F("0")); display.println(s);
      } else {
        display.println(F("INATIVO"));
      }

      display.print(F("Rele Aq  : ")); display.println(AQUECEDOR_ATIVADO ? F("LIGADO") : F("DESLIGADO"));

      if (!listaErros.empty()) {
        display.setCursor(0, 55);
        display.print(F("ERR: ")); display.print(listaErros[0]);
      } else {
        display.setCursor(0, 55);
        display.print(F("B1:Tempo B2:Cfg BE:Aq"));
      }
      break;

    case TELA_TEMPORIZADOR:
      display.println(F("AJUSTE DO TEMPORIZADOR"));
      display.println(F("---------------------"));
      display.println();

      display.setTextSize(2);
      display.setCursor(16, 25);

      // Horas
      if (focadoTempo == 0 && !piscar) display.print(F("  "));
      else {
        if (HR < 10) display.print(F("0"));
        display.print(HR);
      }
      display.print(F(":"));

      // Minutos
      if (focadoTempo == 1 && !piscar) display.print(F("  "));
      else {
        if (MIN < 10) display.print(F("0"));
        display.print(MIN);
      }
      display.print(F(":"));

      // Segundos
      if (focadoTempo == 2 && !piscar) display.print(F("  "));
      else {
        if (SEG < 10) display.print(F("0"));
        display.print(SEG);
      }

      display.setTextSize(1);
      display.setCursor(0, 55);
      display.print(F("B1:Voltar B2:OK BE:Foco"));
      break;

    case TELA_TEMPORIZADOR_CONTANDO:
      {
        uint32_t h = tempoTotalSegundos / 3600;
        uint32_t m = (tempoTotalSegundos % 3600) / 60;
        uint32_t s = tempoTotalSegundos % 60;

        display.print(F("EM EXECUCAO "));
        if (PAUSE) display.println(F("[PAUSADO]"));
        else display.println(AQUECEDOR_ATIVADO ? F("[AQUECENDO]") : F("[AGUARDANDO]"));

        display.setTextSize(2);
        display.setCursor(16, 16);
        if (h < 10) display.print(F("0"));
        display.print(h);
        display.print(F(":"));
        if (m < 10) display.print(F("0"));
        display.print(m);
        display.print(F(":"));
        if (s < 10) display.print(F("0"));
        display.print(s);

        display.setTextSize(1);
        display.setCursor(0, 38);
        display.print(F("Amb:"));
        display.print(TEMP_AMBIENTE, 1);
        display.print(F("C Aq:"));
        display.print(TEMP_AQUECEDOR, 1);
        display.println(F("C"));
        display.print(F("Umidade: "));
        display.print(HUMIDADE_ATUAL, 1);
        display.println(F("%"));

        display.setCursor(0, 55);
        display.print(F("B1:Parar  B2:Pausar"));
      }
      break;

    case TELA_CONFIGURACAO:
      desenharTelaConfiguracao();
      break;

    case TELA_AJUSTE_TEMP:
      display.println(F("-- CALIBRACAO NTC --"));

      display.print(focadoTempo == 0 ? F("> ") : F("  "));
      display.print(F("R_REF: "));
      display.println(cfgvar.R_REF, 1);

      display.print(focadoTempo == 1 ? F("> ") : F("  "));
      display.print(F("BETA : "));
      display.println(cfgvar.BETA, 1);

      display.print(focadoTempo == 2 ? F("> ") : F("  "));
      display.print(F("NTC25: "));
      display.println(cfgvar.NTC_NOMINAL, 1);

      display.setCursor(0, 55);
      display.print(F("B1:Voltar BE:Campo"));
      break;
  }

  display.display();
}

void desenharTelaConfiguracao() {
  display.println(F("--- CONFIGURACOES ---"));

  // Exibe 3 itens simultâneos no carrossel vertical
  for (int i = 0; i < 3; i++) {
    int idx = itemConfigSelecionado + i - 1;  // item anterior, atual e próximo
    if (idx < 0) idx += TOTAL_ITENS_CONFIG;
    if (idx >= TOTAL_ITENS_CONFIG) idx -= TOTAL_ITENS_CONFIG;

    uint8_t lineY = 16 + (i * 12);
    display.setCursor(0, lineY);

    if (i == 1) display.print(modoEdicaoConfig ? F("> ") : F("  "));
    else display.print(F("  "));

    switch (idx) {
      case 0:
        display.print(F("Temp Desej: ")); display.print(cfgvar.TEMP_DESEJADA, 1); display.print(F("C"));
        break;
      case 1:
        display.print(F("Umid Ativ : ")); display.print(cfgvar.HUMIDADE_DE_ATIVACAO, 1); display.print(F("%"));
        break;
      case 2:
        display.print(F("Histerese : ")); display.print(cfgvar.HISTERESE); display.print(F("C"));
        break;
      case 3:
        display.print(F("Int. Umid : ")); display.print(cfgvar.INTERVALO_REATIVACAO_HORAS); display.print(F(" h"));
        break;
      case 4:
        display.print(F("Unidade   : ")); display.print(cfgvar.IS_CELCIUS ? F("Celsius") : F("Fahrenheit"));
        break;
      case 5:
        display.print(F("Debounce  : ")); display.print(cfgvar.DEBOUNCER_TIME); display.print(F(" ms"));
        break;
      case 6:
        display.print(F("Calibrar NTC..."));
        break;
    }

    if (i == 1) {
      display.drawRect(0, lineY - 1, 128, 11, SSD1306_WHITE);
    }
  }

  display.setCursor(0, 55);
  if (modoEdicaoConfig) display.print(F("Gire:Ajustar BE:OK"));
  else display.print(F("B1:Salvar  BE:Editar"));
}


void atualizarLogicaTemporizador() {
  if (aquecimentoAtivo && !PAUSE) {
    unsigned long agora = millis();
    if (agora - ultimoSegundoTimer >= 1000) {
      ultimoSegundoTimer += 1000;
      if (tempoTotalSegundos > 0) {
        tempoTotalSegundos--;
      } else {
        // Fim do tempo do temporizador
        aquecimentoAtivo = false;
        ultimaDesativacaoMs = millis();
        if (telaAtual == TELA_TEMPORIZADOR_CONTANDO) {
          telaAtual = TELA_INICIAL;
        }
      }
    }
  }
}


void carregarConfiguracoes() {
  EEPROM.get(ENDERECO_EEPROM, cfgvar);
  if (cfgvar.assinatura != MAGIC_NUMBER) {
    carregarValoresPadrao();
  }
}

void salvarConfiguracoes() {
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
  cfgvar.INTERVALO_REATIVACAO_HORAS = 5; // Padrão: 5 horas
  cfgvar.IS_CELCIUS = true;
  cfgvar.DEBOUNCER_TIME = 100;
  cfgvar.assinatura = MAGIC_NUMBER;
}

void setup() {
  pinMode(POS_PIN_0, OUTPUT);
  pinMode(POS_PIN_1, OUTPUT);
  pinMode(AQUECEDOR, OUTPUT);
  digitalWrite(AQUECEDOR, LOW);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_SW_ENC, INPUT_PULLUP);
  pinMode(PIN_BT1, INPUT_PULLUP);
  pinMode(PIN_BT2, INPUT_PULLUP);

  Serial.begin(9600);

  carregarConfiguracoes();

  if (!aht.begin()) {
    Serial.println(F("AHT10 ausente!"));
    listaErros.push_back(F("AHT10 ausente"));
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha OLED"));
  }

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ENC_A), isrEncoder, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ENC_B), isrEncoder, FALLING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SW_ENC), isrBotaoEncoder, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BT1), isrBotao1, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BT2), isrBotao2, RISING);
}

unsigned long ultimaAtualizacaoTela = 0;
unsigned long ultimaLeituraSensores = 0;

const unsigned long INTERVALO_TELA = 80;
const unsigned long INTERVALO_SENSORES = 2000;

void loop() {
  unsigned long tempoAtual = millis();

  // 1. Processa entradas do encoder e botões
  processarInputs();

  // 2. Atualiza timer de contagem
  atualizarLogicaTemporizador();

  // 3. Controle e proteção de fundo do aquecedor
  gerenciarAquecimento();

  // 4. Leitura dos sensores (A cada 2s)
  if (tempoAtual - ultimaLeituraSensores >= INTERVALO_SENSORES) {
    ultimaLeituraSensores = tempoAtual;

    float adcA0 = lerADC(PINO_A0);
    float adcA1 = lerADC(PINO_A1);
    TEMP_AMBIENTE = calcularTemperatura(adcA0);
    TEMP_AQUECEDOR = calcularTemperatura(adcA1);

    aht.getEvent(&umidade, &temp);
    TEMP_AH10 = temp.temperature;
    HUMIDADE_ATUAL = umidade.relative_humidity;
    Serial.print("temp A:");
    Serial.print(TEMP_AMBIENTE, 2);
    Serial.print(" Temp AQ:");
    Serial.print(TEMP_AQUECEDOR, 2);
    Serial.print(" Hum:");
    Serial.print(HUMIDADE_ATUAL, 2);
    Serial.print(" ah10:");
    Serial.println(TEMP_AH10, 2);
    // Lógica de aquecimento ligada apenas na contagem sem pausa
  }

  // 5. Renderização do display OLED
  if (tempoAtual - ultimaAtualizacaoTela >= INTERVALO_TELA) {
    ultimaAtualizacaoTela = tempoAtual;
    mostrarTudo();
  }
}
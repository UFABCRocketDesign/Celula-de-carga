// #include <Classes.h> //Biblioteca necessária para o uso do cartão SD
#include "src/lib/Classes.h"

/*
 * Ligação Elétrica da Célula:
 *
 * Red   = V +
 * Black = V -
 * Green = O +
 * White = O -
 */

#define USAR_CELULA (1)      // Se for usar célula de carga
#define USAR_TRANSDUTOR (1)  // Se for usar transdutor de pressão
#define USAR_ENCODER (0)  // Se for usar encoder de rotação

#define USAR_SD (1)  		// Se for usar cartão SD (padrão é usar)

#define USAR_SERIAL (0)  	// Se for exibir na Serial
#define USAR_LORA (1)  		// Se for utiizar o LoRa (monitorar por telemetria)
#define BuZZ (1)         	// Se for usar Buzzer

#define NAME_SD "TE"
/**************************************************************/

#if USAR_TRANSDUTOR
#define TRANSDUTOR_PASCAL (1 && (USAR_TRANSDUTOR))  // Calcular valor de pressão em Pascal
#define TRANSDUTOR_ATM (1 && (USAR_TRANSDUTOR))     // Calcular valor de pressão em atm
#define TRANSDUTOR_BAR (1 && (USAR_TRANSDUTOR))     // Calcular valor de pressão em BAR
#endif                                              // USAR_TRANSDUTOR

#define LoRaHasHeader (1 && (USAR_LORA))
#define LoRaContinuos (0 && (USAR_LORA))             // Se o LoRa deve transmitir continuamente ou só por algum tempo

#define JUST_PLOTTER (0 && (USAR_SERIAL))  // Para ocultar indicação de tempo (usar plotter do arduino)
#define BEEPING (1 && (BuZZ))              // Para sistema apitar quantas partes estão operantes

/**************************************************************/

#if USAR_CELULA
// #define celulaPin A15  //nomeia o pino de entrada como celulaPin
#define celulaPin A2  //nomeia o pino de entrada como celulaPin
#endif                 // USAR_CELULA

#if USAR_TRANSDUTOR
#define transdPin A4  //Pino transdutor
#endif                // USAR_TRANSDUTOR

#if USAR_ENCODER
#define encoderPin 2  // Pino encoder
#define encoderSteps 20 // Espaços no disco do encoder
#endif // USAR_ENCODER

#if BuZZ
#define buzzPin 22   //Pin that the buzzer is connected
#define buzzCmd HIGH //Buzzer is on in high state
#endif               // BuZZ

#if USAR_LORA
#define LORA_DELAY 5 // Atraso em s de cada transmissao do LoRa
#define M0_LORA_PIN 12 // Pinos adicionais do LoRa
#define M1_LORA_PIN 11 // Pinos adicionais do LoRa
#define AUX_LORA_PIN 10 // Pinos adicionais do LoRa

#if !LoRaContinuos
#define LORA_MAX_T 60 // Tempo máximo em s de transmissão LoRa no início do processamento
#endif // LoRaContinuos

#endif // USAR_LORA

/**************************************************************/

#if USAR_CELULA
#define ACEL_G 9.80664999999998  // m/s^2 = 1g
#define kgfToN(X) (X * ACEL_G)   /*Convert Kgf to N*/

#define __Xi 12.5  /*Valor da celula sem corpo de prova*/
#define __Xf 361.5 /*Valor da celula com corpo de prova*/
#define __Yi 0.0   /*Deve conter o valor 0*/
#define __Yf 177.4 /*Peso real do corpo de prova (kg)*/

//7.684 - > 61.7

#define __dX ((__Xf) - (__Xi)) /*Delta X*/
#define __dY ((__Yf) - (__Yi)) /*Delta Y*/

//#define eq(X) (((-__dY)*X+((__Xi)*(__Yf))-(__Xf)-(__Yi))/(-__dX))

#define __angC (__dY / __dX)            /*Coeficiente angular*/
#define __linC (__Yi - (__angC * __Xi)) /*Coeficiente linear*/
//#define __linC (__angC*__Xi)    /*Coeficiente linear*/

#define eq(X) ((__angC * X) + __linC) /*Calib. 2018-12-23*/

#define noiseRangeCell 20
#endif  // USAR_CELULA

#if USAR_TRANSDUTOR
#define noiseRangeTransd 5
#endif  // USAR_TRANSDUTOR

#if BEEPING
Helpful beeper;
#endif  // BEEPING

// Contagem de partes do sistema (serve pra apitar certo)
#define SYSTEM_n ((USAR_SD) + (USAR_CELULA) + (USAR_TRANSDUTOR))

/**************************************************************/

#if USAR_CELULA
int rawCell = 0;  // cria variável cel(Analogic Digital Conversor - ADC) começando em valor 0
int lastCell = 0;
int deltaCell = 0;
double avgKgf = 0, kgfRaw = 0;  // cria variável Kgf começando em valor 0
#endif                       // USAR_CELULA

#if USAR_TRANSDUTOR
int rawTransd;
int lastTransd;
int deltaTransd;
float vSens;
float psiVal;
#if TRANSDUTOR_PASCAL
float pascal;
#endif  //TRANSDUTOR_PASCAL
#if TRANSDUTOR_ATM
float atm;
#endif  //TRANSDUTOR_ATM
#if TRANSDUTOR_BAR
float bar;
#endif  //TRANSDUTOR_BAR
#endif  // USAR_TRANSDUTOR

#if USAR_ENCODER
void encoderInterrupt(void);
volatile unsigned long tAntigo = 0;
volatile unsigned long tAtual = 1;
unsigned long tDiff = 0;
int encoderRPM = 0;
#endif // USAR_ENCODER

float tempo = 0;  // cria variável tempo começando em valor 0

#if USAR_SD
SDCH SDC(53, NAME_SD);  //cria o objeto SDC (cartão de memória SD) e atribui o cs ao pino 53
#endif  // USAR_SD

#if USAR_CELULA
Helpful helpCell;           //cria o objeto help para utilizar a função eachT
MovingAverage avgCell(10);  // Filtragem de dados da célula de carga
MovingAverage avgDeltaCell(5);
#endif  // USAR_CELULA

#if USAR_TRANSDUTOR
Helpful helpTransd;  //cria o objeto help para utilizar a função eachT
MovingAverage avgTransd(10);
MovingAverage avgPSI(10);
#endif               // USAR_TRANSDUTOR

#if USAR_LORA
HardwareSerial &LoRa(Serial2);
Helpful helpLoRa;  //cria o objeto help para utilizar a função eachT
#endif // USAR_LORA

#define holdT 0.1
unsigned short sysC = SYSTEM_n;


/**************************************************************/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

#if USAR_CELULA
  pinMode(celulaPin, INPUT);  //configura o pino A0 como entrada
#endif                        // USAR_CELULA
#if USAR_TRANSDUTOR
  pinMode(transdPin, INPUT);  //configura o pino A1 como entrada
#endif                        // USAR_TRANSDUTOR
#if USAR_ENCODER
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderInterrupt, RISING);
#endif // USAR_ENCODER

#if USAR_LORA
  pinMode(M0_LORA_PIN, OUTPUT);  digitalWrite(M0_LORA_PIN, LOW);
  pinMode(M1_LORA_PIN, OUTPUT);  digitalWrite(M1_LORA_PIN, LOW);
#endif // USAR_LORA

#if USAR_SERIAL
  Serial.begin(115200);
  Serial.println();
#endif  // USAR_SERIAL

#if USAR_LORA
  LoRa.begin(9600);
#endif // USAR_LORA

#if USAR_SD
  SDC.begin();  //inicia o SDcard
#endif          // USAR_SD

#if USAR_SD || USAR_SERIAL
  String dataBuffer = "";

#if USAR_CELULA  // Imprimir a equação de conversão utilizada
  dataBuffer += "Kgf = (\t";
  dataBuffer += String(__angC, 10);
  dataBuffer += "\t) * cell + (\t";
  dataBuffer += String(__linC, 10);
  dataBuffer += "\t)\n";
#endif  //USAR_CELULA

  //---------------------------------------------------------//

  dataBuffer += "tempo"
                "\t";

#if USAR_CELULA
  dataBuffer += "raw.cell"
                "\t";
  dataBuffer += "raw.Kgf"
                "\t";
  dataBuffer += "avg.cell"
                "\t";
  dataBuffer += "avg.Kgf"
                "\t";
  dataBuffer += "avg.N"
                "\t";
#endif  // USAR_CELULA
#if USAR_TRANSDUTOR
  dataBuffer += "raw.tdt"
                "\t";
  dataBuffer += "avgTransd.adc\t";
  dataBuffer += "V.tdt"
                "\t";
  dataBuffer += "psi.tdt"
                "\t";
  dataBuffer += "avgTransd.psi\t";
#if TRANSDUTOR_PASCAL
  dataBuffer += "pascal.tdt"
                "\t";
#endif  // TRANSDUTOR_PASCAL
#if TRANSDUTOR_ATM
  dataBuffer += "atm.tdt"
                "\t";
#endif  // TRANSDUTOR_ATM
#if TRANSDUTOR_BAR
  dataBuffer += "bar.tdt"
                "\t";
#endif  // TRANSDUTOR_BAR
#endif  // USAR_TRANSDUTOR
#if USAR_ENCODER
  dataBuffer += "encoder.RPM"
                "\t";
#endif // USAR_ENCODER

#endif  // USAR_SD || USAR_SERIAL

#if USAR_SD
  if (SDC) {
  sysC++;
  digitalWrite(LED_BUILTIN, LOW);
#if USAR_SERIAL
  Serial.println("SD Begin");
  Serial.println(SDC.getFname());
#endif  // USAR_SERIAL
  SDC.theFile.println(dataBuffer);

  SDC.close();
  } else {
  digitalWrite(LED_BUILTIN, HIGH);
#if USAR_SERIAL
  Serial.println("SD fail");
#endif  // USAR_SERIAL
  }
#endif  // USAR_SD
#if USAR_SERIAL
  Serial.println(dataBuffer);
#endif  // USAR_SERIAL

#if LoRaHasHeader
  String loraBuffer = "Tempo.s\t";
#if USAR_CELULA
  loraBuffer += "avgCell.adc\t";
  loraBuffer += "avgCell.kgf\t";
#endif  // USAR_CELULA
#if USAR_TRANSDUTOR
  loraBuffer += "avgTransd.adc\t";
  loraBuffer += "avgTransd.psi\t";
#endif  // USAR_TRANSDUTOR
#if USAR_ENCODER
  loraBuffer += "vel.rpm\t";
#endif // USAR_ENCODER
  LoRa.println(loraBuffer);
#endif // LoRaHasHeader
#if USAR_LORA && !LoRaContinuos
  helpLoRa.forT(LORA_MAX_T);
#endif // USAR_LORA && !LoRaContinuos

#if BuZZ
  pinMode(buzzPin, OUTPUT);
  digitalWrite(buzzPin, !buzzCmd);
#endif  // BuZZ
#if BEEPING
  beep(sysC);
#endif  // BEEPING
}

/**************************************************************/

void loop() {
  sysC = 0;
#if USAR_CELULA
  rawCell = analogRead(celulaPin);  // lê o valor retornado pela célula de carga
#endif                              // USAR_CELULA
#if USAR_TRANSDUTOR
  rawTransd = analogRead(transdPin);  // lê o valor retornado pelo transdutor de pressão
#endif                                // USAR_TRANSDUTOR
#if USAR_ENCODER
  tDiff = tAtual - tAntigo;
  encoderRPM = 60000000 / (tDiff * encoderSteps);
#endif // USAR_ENCODER

  tempo = float(micros()) / 1000000.0;

#if USAR_CELULA
  avgCell.addValor(rawCell);
  avgKgf = eq(avgCell);  //converte o valor para Kg
  kgfRaw = eq(rawCell);
#endif  // USAR_CELULA

#if USAR_TRANSDUTOR
  avgTransd.addValor(rawTransd);
  vSens = float(rawTransd) * (5.0 / 1023.0);
  psiVal = ((vSens - 0.5) / 4.0) * 500;
  avgPSI.addValor(psiVal);
#if TRANSDUTOR_PASCAL
  pascal = psiVal * 6894.76;
#endif // TRANSDUTOR_PASCAL
#if TRANSDUTOR_ATM
  atm = psiVal / 14.6959;
#endif  // TRANSDUTOR_ATM
#if TRANSDUTOR_BAR
  bar = psiVal / 14.50377;
#endif  // TRANSDUTOR_BAR
#endif  // USAR_TRANSDUTOR


#if USAR_SD || USAR_SERIAL
  String dataBuffer = String(tempo, 3) + "\t";

#if USAR_CELULA
  dataBuffer += String(rawCell) + "\t";
  dataBuffer += String(kgfRaw, 3) + "\t";
  dataBuffer += String(avgCell, 3) + "\t";
  dataBuffer += String(avgKgf, 3) + "\t";
  dataBuffer += String(kgfToN(avgKgf), 3) + "\t";
#endif  // USAR_CELULA
#if USAR_TRANSDUTOR
  dataBuffer += String(rawTransd) + "\t";
    dataBuffer += String(avgTransd) + "\t";
  dataBuffer += String(vSens, 3) + "\t";
  dataBuffer += String(psiVal, 3) + "\t";
    dataBuffer += String(avgPSI, 3) + "\t";
#if TRANSDUTOR_PASCAL
  dataBuffer += String(pascal, 3) + "\t";
#endif  // TRANSDUTOR_PASCAL
#if TRANSDUTOR_ATM
  dataBuffer += String(atm, 3) + "\t";
#endif  // TRANSDUTOR_ATM
#if TRANSDUTOR_BAR
  dataBuffer += String(bar, 3) + "\t";
#endif  // TRANSDUTOR_BAR
#endif  // USAR_TRANSDUTOR
#if USAR_ENCODER
  dataBuffer += String(encoderRPM) + "\t";
#endif // USAR_ENCODER

#endif  // USAR_SD || USAR_SERIAL

#if USAR_SD
  if (SDC) {
    sysC++;
    digitalWrite(LED_BUILTIN, HIGH);
    SDC.theFile.println(dataBuffer);
    SDC.close();
  }
  digitalWrite(LED_BUILTIN, LOW);
#endif  // USAR_SD

#if USAR_SERIAL
  Serial.print(dataBuffer);
#endif  // USAR_SERIAL

#if USAR_LORA
#if !LoRaContinuos
  if(helpLoRa.forT())
#endif // !LoRaContinuos
  if(helpLoRa.eachT(LORA_DELAY)) {
    String loraBuffer = String(tempo, 3) + "\t";
#if USAR_CELULA
    loraBuffer += String(avgCell, 3) + "\t";
    loraBuffer += String(avgKgf, 3) + "\t";
#endif  // USAR_CELULA
#if USAR_TRANSDUTOR
    loraBuffer += String(avgTransd) + "\t";
    loraBuffer += String(avgPSI, 3) + "\t";
#endif  // USAR_TRANSDUTOR
#if USAR_ENCODER
    loraBuffer += String(encoderRPM) + "\t";
#endif // USAR_ENCODER
    LoRa.println(loraBuffer);
  }
#endif // USAR_LORA

#if USAR_CELULA
  deltaCell = abs(rawCell - lastCell);                    // Calcula diferença de medida atual e anterior
  avgDeltaCell = deltaCell;                               // Filtra a diferença
  if (!helpCell.oneTime()) helpCell.comparer(deltaCell);  // Pula a primeira leitura para valor de delta ser coerente

  if (deltaCell > noiseRangeCell) helpCell.forT(holdT * SYSTEM_n * 10);  // "Segura" valor verdadeiro por intervalo de tempo
  if (helpCell.forT()) sysC++;                                           // Se for valor verdadeiro, adiciona ao contador de beeps

#if USAR_SERIAL
  Serial.print('\t');
  Serial.print(deltaCell);
  Serial.print('\t');
  Serial.print(helpCell.getMax(), 1);
  Serial.print('\t');
  Serial.print(avgDeltaCell, 3);
#endif  // USAR_SERIAL

  lastCell = rawCell;
#endif  // USAR_CELULA


#if USAR_TRANSDUTOR
  deltaTransd = abs(rawTransd - lastTransd);                    // Calcula diferença de medida atual e anterior
  if (!helpTransd.oneTime()) helpTransd.comparer(deltaTransd);  // Pula a primeira leitura para valor de delta ser coerente

  if (deltaTransd > noiseRangeTransd) helpTransd.forT(holdT * SYSTEM_n * 10);  // "Segura" valor verdadeiro por intervalo de tempo
  if (helpTransd.forT()) sysC++;                                               // Se for valor verdadeiro, adiciona ao contador de beeps

#if USAR_SERIAL
  Serial.print('\t');
  Serial.print(helpTransd.getMax(), 1);
#endif  // USAR_SERIAL

  lastTransd = rawTransd;
#endif  // USAR_CELULA

#if USAR_SERIAL
  Serial.print('\t');
  Serial.print(sysC);
  Serial.print('\n');
#endif  // USAR_SERIAL
#if BEEPING
  beep(sysC);
#endif  // BEEPING
}

/**************************************************************/

#if USAR_ENCODER
void encoderInterrupt(void) {
  tAntigo = tAtual;
  tAtual = micros();
}
#endif // USAR_ENCODER

/**************************************************************/

#if BEEPING
inline void beep(unsigned int N) {
  if (beeper.eachT(holdT * SYSTEM_n * 10) || beeper.oneTime()) {
  beeper.mem = buzzCmd;
  beeper.counterReset();
  beeper.forT(holdT);
  }
  if (beeper.getCount() < (N + 1) * 2)
  if (!beeper.forT()) {
    digitalWrite(buzzPin, beeper.mem);
    beeper.mem = !beeper.mem;
    beeper.counter();
    beeper.forT(holdT);
  }
}

inline void beep() {
  digitalWrite(buzzPin, !buzzCmd);
  beeper.counterReset();
}
#endif  // BEEPING
#include <Classes.h> //Biblioteca necessária para o uso do cartão SD

/*
 * Ligação Elétrica da Célula:
 *
 * Red   = V +
 * Black = V -
 * Green = O +
 * White = O -
 */

#define USAR_CELULA (1)         // Se for usar célula de carga
#define USAR_TRANSDUTOR (1)     // Se for usar transdutor de pressão

#define USAR_SD (1)             // Se for usar cartão SD (padrão é usar)

#define USAR_SERIAL (1)         // Se for exibir na Serial
#define BuZZ (1)                // Se for usar Buzzer

/**************************************************************/

#if USAR_TRANSDUTOR
#define TRANSDUTOR_PASCAL (1 && (USAR_TRANSDUTOR))  // Calcular valor de pressão em Pascal
#define TRANSDUTOR_ATM (1 && (USAR_TRANSDUTOR))     // Calcular valor de pressão em atm
#define TRANSDUTOR_BAR (1 && (USAR_TRANSDUTOR))     // Calcular valor de pressão em BAR
#endif // USAR_TRANSDUTOR

#define JUST_PLOTTER (0 && (USAR_SERIAL))    // Para ocultar indicação de tempo (usar plotter do arduino)
#define BEEPING (1 && (BuZZ))             // Para sistema apitar quantas partes estão operantes

/**************************************************************/

#if USAR_CELULA
#define celulaPin A15 //nomeia o pino de entrada como celulaPin
#endif // USAR_CELULA

#if USAR_TRANSDUTOR
#define transdPin A3 //Pino transdutor
#endif // USAR_TRANSDUTOR

#if BuZZ
#define buzzPin 22              //Pin that the buzzer is connected
#define buzzCmd LOW             //Buzzer is on in high state
#endif // BuZZ

/**************************************************************/

#if USAR_CELULA
#define ACEL_G 9.80664999999998 // m/s^2 = 1g
#define kgfToN(X) (X*ACEL_G) /*Convert Kgf to N*/

#define __Xi 18.0 /*Valor da celula sem corpo de prova*/
#define __Xf 257.0  /*Valor da celula com corpo de prova*/
#define __Yi 0.0   /*Deve conter o valor 0*/
#define __Yf 100.8  /*Peso real do corpo de prova (kg)*/

//7.684 - > 61.7

#define __dX ((__Xf) - (__Xi))  /*Delta X*/
#define __dY ((__Yf) - (__Yi))  /*Delta Y*/

//#define eq(X) (((-__dY)*X+((__Xi)*(__Yf))-(__Xf)-(__Yi))/(-__dX))

#define __angC (__dY/__dX)      /*Coeficiente angular*/
#define __linC (__Yi-(__angC*__Xi))   /*Coeficiente linear*/
//#define __linC (__angC*__Xi)    /*Coeficiente linear*/

#define eq(X) ((__angC*X)+__linC) /*Calib. 2018-12-23*/

#define noiseRangeCell 20
#endif // USAR_CELULA

#if USAR_TRANSDUTOR
#define noiseRangeTransd 1
#endif // USAR_TRANSDUTOR

#if BEEPING
Helpful beeper;
#define holdT .1
#endif // BEEPING

// Contagem de partes do sistema (serve pra apitar certo)
#define SYSTEM_n ((USAR_SD) + (USAR_CELULA) + (USAR_TRANSDUTOR))

/**************************************************************/

#if USAR_CELULA
int rawCell = 0; // cria variável cel(Analogic Digital Conversor - ADC) começando em valor 0
int lastCell = 0;
int deltaCell = 0;
double kgf = 0, kgfRaw = 0; // cria variável Kgf começando em valor 0
#endif // USAR_CELULA

#if USAR_TRANSDUTOR
int rawTransd;
int lastTransd;
int deltaTransd;
float vSens;
float psiVal;
#if TRANSDUTOR_PASCAL
float pascal;
#endif //TRANSDUTOR_PASCAL
#if TRANSDUTOR_ATM
float atm;
#endif //TRANSDUTOR_ATM
#if TRANSDUTOR_BAR
float bar;
#endif //TRANSDUTOR_BAR
#endif // USAR_TRANSDUTOR

float tempo = 0; // cria variável tempo começando em valor 0

#if USAR_SD
SDCH SDC(53, "cell"); //cria o objeto SDC (cartão de memória SD) e atribui o cs ao pino 53
#endif // USAR_SD

#if USAR_CELULA
Helpful helpCell; //cria o objeto help para utilizar a função eachT
MovingAverage avgCell(20); // Filtragem de dados da célula de carga
MovingAverage avgDeltaCell(5);
#endif // USAR_CELULA

#if USAR_TRANSDUTOR
Helpful helpTransd; //cria o objeto help para utilizar a função eachT
#endif // USAR_TRANSDUTOR

unsigned short sysC = SYSTEM_n;


/**************************************************************/

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#if USAR_SERIAL
  Serial.begin(250000);
  Serial.println();

#if USAR_CELULA // Imprimir a equação de conversão utilizada
  Serial.print("Kgf = (");
  Serial.print(__angC, 10);
  Serial.print(") * cell + (");
  //Serial.print(__linC, 10);
  Serial.print(__linC, 10);
  Serial.print(")");
  Serial.println();
#endif //USAR_CELULA
#endif // USAR_SERIAL
#if USAR_SD
    SDC.begin();                 //inicia o SDcard
#endif // USAR_SD
#if USAR_CELULA
    pinMode(celulaPin, INPUT);   //configura o pino A0 como entrada
#endif // USAR_CELULA
#if USAR_TRANSDUTOR
    pinMode(transdPin, INPUT);   //configura o pino A1 como entrada
#endif // USAR_TRANSDUTOR

#if USAR_SD
  if (SDC) {
    sysC++;
  digitalWrite(LED_BUILTIN, LOW);
#if USAR_SERIAL
    Serial.println("SD Begin");
    Serial.println(SDC.getFname());
#endif // USAR_SERIAL
    //---------------------------------------------------------//]
#if USAR_CELULA // Imprimir a equação de conversão utilizada
    SDC.theFile.print("Kgf = (");
    SDC.tab();
    SDC.theFile.print(__angC, 9);
    SDC.tab();
    SDC.theFile.print(") * cell +(");
    SDC.tab();
    //SDC.theFile.print(__linC, 9);
    SDC.theFile.print(__linC,9);
    SDC.theFile.println("\t)");
#endif // USAR_CELULA
    //---------------------------------------------------------//
    SDC.theFile.print("tempo");
#if USAR_CELULA
    SDC.tab();
    SDC.theFile.print("raw.cell");
    SDC.tab();
    SDC.theFile.print("raw.Kgf");
    SDC.tab();
    SDC.theFile.print("avg.cell");  //Cel filtrado
    SDC.tab();
    SDC.theFile.print("avg.Kgf");
    SDC.tab();
    SDC.theFile.print("avg.N");
#endif // USAR_CELULA
#if USAR_TRANSDUTOR
    SDC.tab();
    SDC.theFile.print("raw.tdt");
    SDC.tab();
    SDC.theFile.print("V.tdt");
    SDC.tab();
    SDC.theFile.print("psi.tdt");
    #if TRANSDUTOR_PASCAL
      SDC.tab();
      SDC.theFile.print("pascal.tdt");
    #endif // TRANSDUTOR_PASCAL
    #if TRANSDUTOR_ATM
      SDC.tab();
      SDC.theFile.print("atm.tdt");
    #endif // TRANSDUTOR_ATM
    #if TRANSDUTOR_BAR
      SDC.tab();
      SDC.theFile.print("bar.tdt");
    #endif // TRANSDUTOR_BAR
#endif // USAR_TRANSDUTOR
    SDC.theFile.println();
    SDC.close();
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
#if USAR_SERIAL
    Serial.println("SD fail");
#endif // USAR_SERIAL
  }
#endif // USAR_SD
#if BuZZ
  pinMode(buzzPin, OUTPUT);
  digitalWrite(buzzPin, !buzzCmd);
#endif // BuZZ
#if BEEPING
  beep(sysC);
#endif // BEEPING
}

/**************************************************************/

void loop()
{
  sysC  = 0;
#if USAR_CELULA
  rawCell = analogRead(celulaPin); // lê o valor retornado pela célula de carga
#endif // USAR_CELULA
#if USAR_TRANSDUTOR
  rawTransd = analogRead(transdPin); // lê o valor retornado pelo transdutor de pressão
#endif // USAR_TRANSDUTOR
  tempo = float(micros()) / 1000000.0;

#if USAR_CELULA
  avgCell = rawCell;
  kgf = eq(avgCell);        //converte o valor para Kg
  kgfRaw = eq(rawCell);
#endif // USAR_CELULA

#if USAR_TRANSDUTOR
  vSens = float(rawTransd)*(5.0/1023.0);
  psiVal = ((vSens - 0.5)/4.0) * 500;

  #if TRANSDUTOR_PASCAL
    pascal = psiVal * 6894.76;
  #endif // TRANSDUTOR_PASCAL
  #if TRANSDUTOR_ATM
    atm = psiVal / 14.6959;
  #endif // TRANSDUTOR_ATM
  #if TRANSDUTOR_BAR
    bar = psiVal / 14.50377;
  #endif // TRANSDUTOR_BAR
#endif // USAR_TRANSDUTOR

#if USAR_SD
  if (SDC) {
    sysC++;
    digitalWrite(LED_BUILTIN, HIGH);
    SDC.theFile.print(tempo, 3);
  #if USAR_CELULA
    SDC.tab();
    SDC.theFile.print(rawCell);
    SDC.tab();
    SDC.theFile.print(kgfRaw, 3);
    SDC.tab();
    SDC.theFile.print(avgCell, 3);
    SDC.tab();
    SDC.theFile.print(kgf, 3);
    SDC.tab();
    SDC.theFile.print(kgfToN(kgf), 3);
  #endif // USAR_CELULA
  #if USAR_TRANSDUTOR
    SDC.tab();
    SDC.theFile.print(rawTransd);
    SDC.tab();
    SDC.theFile.print(vSens, 3);
    SDC.tab();
    SDC.theFile.print(psiVal, 3);
    #if TRANSDUTOR_PASCAL
      SDC.tab();
      SDC.theFile.print(pascal, 3);
    #endif // TRANSDUTOR_PASCAL
    #if TRANSDUTOR_ATM
      SDC.tab();
      SDC.theFile.print(atm, 3);
    #endif // TRANSDUTOR_ATM
    #if TRANSDUTOR_BAR
      SDC.tab();
      SDC.theFile.print(bar, 3);
    #endif // TRANSDUTOR_BAR
#endif // USAR_TRANSDUTOR
    SDC.theFile.println();
    SDC.close();
  }
  digitalWrite(LED_BUILTIN, LOW);
#endif // USAR_SD


#if USAR_SERIAL
  #if !JUST_PLOTTER
    Serial.print(tempo, 3);
  #endif // !JUST_PLOTTER
  #if USAR_CELULA
    Serial.print('\t');
    Serial.print(rawCell);
    Serial.print('\t');
    #if !JUST_PLOTTER
      Serial.print(kgfRaw, 6);
      Serial.print('\t');
    #endif // !JUST_PLOTTER
    Serial.print(avgCell, 6);
    Serial.print('\t');
    #if !JUST_PLOTTER
      Serial.print(kgf, 6);
      Serial.print('\t');
      Serial.print(kgfToN(kgf), 6);
    #endif // !JUST_PLOTTER
  #endif  // USAR_CELULA
  #if USAR_TRANSDUTOR
    Serial.print('\t');
    Serial.print(rawTransd);
    Serial.print('\t');
    Serial.print(vSens, 3);
    Serial.print('\t');
    Serial.print(psiVal, 3);
    #if TRANSDUTOR_PASCAL
      Serial.print('\t');
      Serial.print(pascal, 3);
    #endif // TRANSDUTOR_PASCAL
    #if TRANSDUTOR_ATM
      Serial.print('\t');
      Serial.print(atm, 3);
    #endif // TRANSDUTOR_ATM
    #if TRANSDUTOR_BAR
      Serial.print('\t');
      Serial.print(bar, 3);
    #endif // TRANSDUTOR_BAR
  #endif // USAR_TRANSDUTOR
#endif // USAR_SERIAL

#if USAR_CELULA
  deltaCell = abs(rawCell - lastCell); // Calcula diferença de medida atual e anterior
  avgDeltaCell = deltaCell; // Filtra a diferença
  if(!helpCell.oneTime()) helpCell.comparer(deltaCell); // Pula a primeira leitura para valor de delta ser coerente

  if(deltaCell > noiseRangeCell) helpCell.forT(holdT * SYSTEM_n * 4); // "Segura" valor verdadeiro por intervalo de tempo
  if(helpCell.forT()) sysC++; // Se for valor verdadeiro, adiciona ao contador de beeps

  #if USAR_SERIAL
    Serial.print('\t');
    Serial.print(deltaCell);
    Serial.print('\t');
    Serial.print(helpCell.getMax(),1);
    Serial.print('\t');
    Serial.print(avgDeltaCell,3);
  #endif // USAR_SERIAL

  lastCell = rawCell;
#endif // USAR_CELULA


#if USAR_TRANSDUTOR
  deltaTransd = abs(rawTransd - lastTransd); // Calcula diferença de medida atual e anterior
  if(!helpTransd.oneTime()) helpTransd.comparer(deltaTransd); // Pula a primeira leitura para valor de delta ser coerente

  if(deltaTransd > noiseRangeTransd) helpTransd.forT(holdT * SYSTEM_n * 4); // "Segura" valor verdadeiro por intervalo de tempo
  if(helpTransd.forT()) sysC++; // Se for valor verdadeiro, adiciona ao contador de beeps

  #if USAR_SERIAL
    Serial.print('\t');
    Serial.print(helpTransd.getMax(),1);
  #endif // USAR_SERIAL

  lastTransd = rawTransd;
#endif // USAR_CELULA

#if USAR_SERIAL
  Serial.print('\t');
  Serial.print(sysC);
  Serial.print('\n');
#endif // USAR_SERIAL
  #if BEEPING
    beep(sysC);
  #endif // BEEPING
}

/**************************************************************/

#if BEEPING
inline void beep(unsigned int N)
{
  if (beeper.eachT(holdT*SYSTEM_n * 4) || beeper.oneTime())
  {
    beeper.mem = buzzCmd;
    beeper.counterReset();
    beeper.forT(holdT);
  }
  if (beeper.getCount() < (N + 1) * 2) if (!beeper.forT())
  {
    digitalWrite(buzzPin, beeper.mem);
    beeper.mem = !beeper.mem;
    beeper.counter();
    beeper.forT(holdT);
  }
}

inline void beep()
{
  digitalWrite(buzzPin, !buzzCmd);
  beeper.counterReset();
}
#endif // BEEPING


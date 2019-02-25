#include <Classes.h> //Biblioteca necessária para o uso do cartão SD

#define analogPin A15 //nomeia o pino de entrada como analogPin      

#define ACEL_G 9.80664999999998 
#define kgfToN(X) (X*ACEL_G) /*Convert Kgf to N*/

#define __Xi 29.8 /*Valor da celula sem corpo de prova*/
#define __Xf 615.0  /*Valor da celula com corpo de prova*/
#define __Yi 0.0   /*Deve conter o valor 0*/
#define __Yf 88.2  /*Peso real do corpo de prova (kg)*/

//7.684 - > 61.7

#define __dX ((__Xf) - (__Xi))  /*Delta X*/
#define __dY ((__Yf) - (__Yi))  /*Delta Y*/

//#define eq(X) (((-__dY)*X+((__Xi)*(__Yf))-(__Xf)-(__Yi))/(-__dX))

#define __angC (__dY/__dX)      /*Coeficiente angular*/
#define __linC (__Yi-(__angC*__Xi))   /*Coeficiente linear*/
//#define __linC (__angC*__Xi)    /*Coeficiente linear*/

#define eq(X) ((__angC*X)+__linC) /*Calib. 2018-12-23*/

#define noiseRange 20

#define SERIAL 1
#define JUST_PLOTTER (0 && (SERIAL))
#define BuZZ (1)
#define BEEPING (BuZZ && 1)

#if BuZZ
#define buzzPin A0              //Pin that the buzzer is connected
#define buzzCmd LOW             //Buzzer is on in high state
#endif // BuZZ

#if BEEPING
#define holdT .1
Helpful beeper;
#endif // BEEPING

#define SYSTEM_n 5

int cel = 0; // cria variável cel(Analogic Digital Conversor - ADC) começando em valor 0
double kgf = 0, kgfRaw = 0; // cria variável Kgf começando em valor 0
float tempo = 0; // cria variável tempo começando em valor 0
SDCH SDC(53, "cell"); //cria o objeto SDC (cartão de memória SD) e atribui o cs ao pino 53
Helpful help; //cria o objeto help para utilizar a função eachT

MovingAverage AVG(20);
MovingAverage RNG(5);

unsigned short sysC = 0;
int lastRead = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#if SERIAL
  Serial.begin(250000);
  Serial.println();
  Serial.print("Kgf = (");
  Serial.print(__angC, 10);
  Serial.print(") * cell + (");
  //Serial.print(__linC, 10);
  Serial.print(__linC, 10);
  Serial.print(")");
  Serial.println();
#endif
    SDC.begin();                 //inicia o SDcard
    pinMode(analogPin, INPUT);   //configura o pino A0 como entrada

  if (SDC) {
  digitalWrite(LED_BUILTIN, LOW);
#if SERIAL
    Serial.println("SD Begin");
    Serial.println(SDC.getFname());
#endif
    //---------------------------------------------------------//
    SDC.theFile.print("Kgf = (");
    SDC.tab();
    SDC.theFile.print(__angC, 9);
    SDC.tab();
    SDC.theFile.print(") * cell +(");
    SDC.tab();
    //SDC.theFile.print(__linC, 9);
    SDC.theFile.print(__linC,9);
    SDC.theFile.println(")");
    //---------------------------------------------------------//
    SDC.theFile.print("tempo");
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
    SDC.theFile.println();
    SDC.close();
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
#if SERIAL
    Serial.println("SD fail");
#endif
  }
#if BuZZ
  pinMode(buzzPin, OUTPUT);
  digitalWrite(buzzPin, !buzzCmd);
#endif // BuZZ
#if BEEPING
  beep();
#endif // BEEPING
}


void loop()
{
  sysC  = 0;
  cel = analogRead(analogPin); // lê o valor retornado pela célula de carga
  tempo = float(micros()) / 1000000.0;

  AVG = cel;
  kgf = eq(AVG);        //converte o valor para Kg
  kgfRaw = eq(cel);

  if (SDC) {
    sysC++;
    digitalWrite(LED_BUILTIN, HIGH);
    SDC.theFile.print(tempo, 3);
    SDC.tab();
    SDC.theFile.print(cel);
    SDC.tab();
    SDC.theFile.print(kgfRaw, 3);
    SDC.tab();
    SDC.theFile.print(AVG, 3);
    SDC.tab();
    SDC.theFile.print(kgf, 3);
    SDC.tab();
    SDC.theFile.print(kgfToN(kgf), 3);
    SDC.theFile.println();
    SDC.close();
  }
  digitalWrite(LED_BUILTIN, LOW);
#if SERIAL
#if !JUST_PLOTTER
  Serial.print(tempo, 3);
  Serial.print('\t');
#endif
  Serial.print(cel);
  Serial.print('\t');
#if !JUST_PLOTTER
  Serial.print(kgfRaw, 6);
  Serial.print('\t');
#endif
  Serial.print(AVG, 6);
  Serial.print('\t');
#if !JUST_PLOTTER
  Serial.print(kgf, 6);
  Serial.print('\t');
  Serial.print(kgfToN(kgf), 6);
#endif
  Serial.print('\t');
#endif

  RNG = abs(cel - lastRead);
  if(!help.oneTime())help.comparer(abs(cel - lastRead));

  if(abs(cel - lastRead)>noiseRange) help.forT(holdT*SYSTEM_n * 4);
  if(help.forT()) sysC++;
#if SERIAL
  Serial.print(abs(cel - lastRead));
  Serial.print('\t');
  Serial.print(help.getMax(),1);
  Serial.print('\t');
  Serial.print(RNG,3);
  Serial.print('\t');
  Serial.print(sysC);
#endif
  beep(sysC);
  lastRead = cel;

#if SERIAL
  Serial.print('\n');
#endif
}

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


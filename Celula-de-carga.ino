#include <Classes.h> //Biblioteca necessária para o uso do cartão SD

#define analogPin A0 //nomeia o pino A0 como analogPin      

#define ACEL_G 9.80664999999998 

#define __Xi 100.7  /*Valor da celula sem corpo de prova*/
#define __Xf 507.7  /*Valor da celula com corpo de prova*/
#define __Yi 0.0    /*Deve conter o valor 0*/
#define __Yf 5.459  /*Peso real do corpo de prova (kg)*/

#define __dX ((__Xf) - (__Xi))  /*Delta X*/
#define __dY ((__Yf) - (__Yi))  /*Delta Y*/

#define __angC (__dY/__dX)      /*Coeficiente angular*/
#define __linC (__angC*__Xi)    /*Coeficiente linear*/

#define eq(X) ((__angC*X)- __linC) /*Calb. 2018-07-07*/

#define SERIAL 1

int cel = 0; // cria variável cel(Analogic Digital Conversor - ADC) começando em valor 0
double kg = 0; // cria variável Kg começando em valor 0
float tempo = 0; // cria variável tempo começando em valor 0
SDCH SDC(53, "cell"); //cria o objeto SDC (cartão de memória SD) e atribui o cs ao pino 53
Helpful help; //cria o objeto help para utilizar a função eachT

MovingAverage AVG(20);
MovingAverage AVG2(20);
MovingAverage AVG3(20);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#if SERIAL
  Serial.begin(115200);
  Serial.println();
  Serial.print(__angC, 10);
  Serial.print(" * x - ");
  Serial.print(__linC, 10);
  Serial.print(" = Kg");
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
    SDC.theFile.print("tempo");
    SDC.tab();
    SDC.theFile.print("vlr.cel");
    SDC.tab();
    SDC.theFile.print("vlr.Kg");
    SDC.tab();
    SDC.theFile.print("vlr.N");
    SDC.tab();
    SDC.theFile.print("avg.cell");  //Cel filtrado
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
}


void loop()
{

  cel = analogRead(analogPin); // lê o valor retornado pela célula de carga
  tempo = float(micros()) / 1000000.0;
//  AVG3 = AVG2 = AVG = cel;
  AVG3 = cel;
  kg = eq(AVG3);        //converte o valor para Kg

  if (SDC) {
    digitalWrite(LED_BUILTIN, HIGH);
    SDC.theFile.print(tempo, 3);
    SDC.tab();
    SDC.theFile.print(cel);
    SDC.tab();
    SDC.theFile.print(kg, 3);
    SDC.tab();
    SDC.theFile.print(kg*ACEL_G, 3);
    SDC.tab();
    SDC.theFile.print(AVG3, 3);
    SDC.theFile.println();
    SDC.close();
  }
  digitalWrite(LED_BUILTIN, LOW);
#if SERIAL
  Serial.print(tempo, 3);
  Serial.print('\t');
  Serial.print(cel);
  Serial.print('\t');
//  Serial.print(AVG, 6);
//  Serial.print('\t');
//  Serial.print(AVG2, 6);
//  Serial.print('\t');
  Serial.print(AVG3, 6);
  Serial.print('\t');
  Serial.print(kg, 6);
  Serial.print('\t');
  Serial.print(kg*ACEL_G, 6);
  Serial.print('\n');
#endif
}


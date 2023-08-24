#include <Wire.h>
#include <SoftwareSerial.h>

//---------------------------------------------------------------------------------------
//Definindo existencia: 
//(1 = Existe,  0 = Não existe). 
//caso "não exista", toda a parte relacionada a essa existencia será comentada,
// de forma a não aparecer no monitor serial e nem pesar no processamento do arduino. 

#define EXIST_UNO 0 // por enqunto define se vai utilizar arduino UNO ou mega
#define EXIST_MEGA (!EXIST_UNO)

#define EXIST_BLUE 1 // existencia do modulo bluetooth HC06

#define EXIST_FILTRO 0 // existencia de filtros

#define EXIST_VISAO 0 // existencia do modulo bluetooth HC06
#define EXIST_VISAO_FILTRO (EXIST_FILTRO && EXIST_VISAO && 1) //existencia de filtro nos dados da visão computacional
#define EXIST_VISAO_ORIGINAL (EXIST_VISAO && 1)

#define EXIST_MOTOR_DC 1 // existencia dos motores dc
#define EXIST_CALIBRA_PWM_MANUAL (EXIST_BLUE && EXIST_MOTOR_DC && 1) // existencia da função de calibrar o pwm minimo e maximo manualmente 
#define EXIST_ENCODER 0 // existencia dos enconders
#define EXIST_CALIBRA_PWM (EXIST_ENCODER && 1) // existencia da função de calibrar o pwm minimo e maximo automatico

#define EXIST_SERVO 1 // existencia do servo motoror
#define EXIST_CALIBRA_SERVO (EXIST_SERVO && EXIST_BLUE && 1) // existencia da função de calibrar manualmente o servo motor

#define EXIST_CONTROLE_REMOTO (EXIST_BLUE && EXIST_MOTOR_DC && EXIST_SERVO && 1) // existencia de controlar o carro remotamente

#define EXIST_Ultrassonico 1  // existencia do sensor ultrassonico
#define EXIST_Ultrassonico_FILTRO (EXIST_FILTRO && EXIST_Ultrassonico && 1) // existencia do filtro para o sensor ultrassonico
#define EXIST_Ultrassonico_ORIGINAL (EXIST_Ultrassonico && 1)  //define a existencia do print do valor original


#define EXIST_DADOS 1 // existencia de dados para print
#define EXIST_MEDIDA (EXIST_DADOS && 0) // existencia de unidade de media para os dados 
#define EXIST_AJUSTE_GRAFICO (EXIST_DADOS && 0)

//-----------------------------------------------------------------------------
// Definindo pinos do arduino:
#define PIN_MD1 12 // pino 1 de controle do motor direito (dominante)
#define PIN_MD2 11 // pino 2 de controle do motor direito
#define PIN_ME1 10 // pino 1 de controle do motor esquerdo (dominante)
#define PIN_ME2 9  // pino 2 de controle do motor esquerdo


#define PIN_EN_DA 18 // pino A de controle do enconder dieito para arduino MEGA
#define PIN_EN_DB 19 // pino B de controle do enconder dieito para arduino MEGA
#if EXIST_MEGA 
#define PIN_EN_EA 20 // pino A de controle do enconder esquerdo para arduino MEGA
#define PIN_EN_EB 21 // pino B de controle do enconder esquerdo para arduino MEGA
#endif // EXIST_MEGA 


#define PIN_SERVO 8 // pino de controle do servo motor 

#define PIN_TRIG_1 7 // pino trig do ultrassonico 1
#define PIN_ECHO_1 6 // pino echo do ultrassonico 1
#if EXIST_MEGA 
#define PIN_TRIG_2 24 // pino trig do ultrassonico 2
#define PIN_ECHO_2 26 // pino echo do ultrassonico 2
#define PIN_TRIG_3 30 // pino trig do ultrassonico 3
#define PIN_ECHO_3 28 // pino echo do ultrassonico 3
#endif // EXIST_MEGA 


#if EXIST_UNO
#if EXIST_BLUE
SoftwareSerial HC06(5, 4); // define os pinos TX, RX do bluetooth para arduino UNO
#endif // EXIST_BLUE
#endif // EXIST_UNO
#if EXIST_MEGA 
#if EXIST_BLUE
SoftwareSerial HC06(50, 51); // define os pinos TX, RX do bluetooth para arduino MEGA
#endif // EXIST_BLUE
#endif // EXIST_MEGA

//-----------------------------------------------------------------------------
// Definindo constantes:
#define PWM_MAXIMO 140  // pwm maximo para fazer o motor girar (0 a 225)
#define PWM_MINIMO 80  // pwm minimo para fazer o motor girar (0 a 225)
#define VEL_MAX 6 // relocidade maxima (m/s) que o carro deve atingir 
#define RAIO_RODA 0.0285 // raio da roda em metros
#define NUM_PULSO_VOLTA 1440.0 // numero de opulsos necessarios para o enconder contabilizar 1 volta
// 1440.0 = 1 volta | 2880.0 = 2 voltas


// Para conseguir os valores aseguir, faça uma calibração:
// original = 0 
#define ANGULO_INICIAL 0 // angulo real inicial do servo para deixar as rodas retas (real = angulo interno do servo)
#define ANGULO_ZERO 50 // angulo real que sera considerado zero
// original = 180 
#define ANGULO_MAX 102 // angulo real maximo que o servo deve atingir
// original = 0 
#define ANGULO_MIN 0 // angulo minimo real que o servo deve atingir
#define SERVO_SINAL_MIN 500 //sinal em microsegundos do angulo minimo do servo
#define SERVO_SINAL_MAX 2400 //sinal em microsegundos do angulo maximo do servo

#define INTERVALO_MEDIA_HCSR04 25
#define NUMERO_FILTROS_HCSR04 1
#define DISTANCIA_PARAR 8 //distancia minima (em cm) para o carro parar

#define INTERVALO_MEDIA_VISAO 20
#define NUMERO_FILTROS_VISAO 2

//-----------------------------------------------------------------------------
// variaveis globais:
int switch_case = 0;  // variavel que controla os casos do switch case

int pwm = PWM_MAXIMO; // pwm inicial
int pwm_min = PWM_MINIMO; // pwm maximo que o carro irá atingir
int pwm_max = PWM_MAXIMO; // pwm minimo que o carro precisa para andar
int estado_motor; // indica por meio de 0 ou 1 se o motor está ligado ou desligado


int angulo_servo = ANGULO_INICIAL; // armazena o angulo real do servo motor 
int angulo_zero = ANGULO_ZERO; // armazena apenas o angulo que irá definir o ponto zero 
int angulo_maximo = ANGULO_MAX; // armazena o angulo real maximo que o servo consegue abrir
int angulo_minimo = ANGULO_MIN; // armazena o angulo real maximo que o servo consegue abrir
#if EXIST_VISAO
int angulo_visao, angulo_visao_real, angulo_visao_f; // armazena o angulo dado pela visão computacional
int esquerda, direita, offset;
#endif //EXIST_VISAO

bool obstaculo = false; // armazena a indicação de obstaculo no caminho do sensor 3
bool obstaculo_1 = false; // armazena a indicação de obstaculo no caminho do sensor 1
bool obstaculo_2 = false; // armazena a indicação de obstaculo no caminho do sensor 2
bool obstaculo_3 = false; // armazena a indicação de obstaculo no caminho do sensor 3
bool trava_tempo_V = true;
bool trava_tempo_B = true;

double vel_md, vel_me; // armazenam a velocidade em (m/s) dos motores direito e esquerdo respectivamente
double dist_total; // armazenam a distancia total percorrida

String dados_print_HC06 = " ";  // armazena os dados que serão printado no bluetooth
String dados_print_PC = " ";  // armazena os dados que serão printado no monitor serial
String dado_menu = "0";
String dados_visao = " ";

#if EXIST_BLUE
char msg_blue; // armazena os dados recebido do bluetooth ou monitor serial do pc 
#endif // EXIST_BLUE

#if EXIST_Ultrassonico 
const float velocidadeSom = 0.00034029; // velocidade do som em metros/microsegundo
float distancia_1, distancia_1f, distancia_2, distancia_2f, distancia_3, distancia_3f;
int detec;
#endif // EXIST_Ultrassonico

/*******************************************************************/
#if EXIST_MOTOR_DC 
// classe responsavel pelo setup e orientação dos motores:
class Motores {
  int pin_m1;
  int pin_m2;

 public:
  Motores(int pin_1, int pin_2){
  pin_m1 = pin_1;
  pin_m2 = pin_2;
  pinMode(pin_m1, OUTPUT);  
  pinMode(pin_m2, OUTPUT);  
  }

  void frente(int pwm) {
  analogWrite(pin_m1,pwm);
  digitalWrite(pin_m2,LOW);
  estado_motor = 1;
  }

  void tras(int pwm) {
  digitalWrite(pin_m1,LOW);
  analogWrite(pin_m2,pwm);
  estado_motor = -1;
  }

  void para() {
  digitalWrite(pin_m1,LOW);
  digitalWrite(pin_m2,LOW);
  estado_motor = 0;
  }
};
Motores motor_direito(PIN_MD1, PIN_MD2);
Motores motor_esquerdo(PIN_ME1, PIN_ME2);
#endif // EXIST_MOTOR_DC 

//-----------------------------------------------------------------------------
// classe responsavel pela contagem de tempo em millis():
class Contador_tempo {
  unsigned long intervalo;
  unsigned long ultima_atualizacao;
  bool trava_chamado;
  
 public:
  Contador_tempo(unsigned long intervalo_ms){
    intervalo = intervalo_ms;
    trava_chamado = true;
  }
  
  bool atingiu_tempo() {
    unsigned long tempo_atual = millis();
    if(trava_chamado){
      ultima_atualizacao = tempo_atual;
      trava_chamado = false;
    }   
    if(tempo_atual - ultima_atualizacao >= intervalo){
      trava_chamado = true;      
      return true;
    }
    return false;
  }
};
Contador_tempo um_segundo_enconder(1000); 

//-----------------------------------------------------------------------------
#if EXIST_SERVO
//classe para controle de servos
class Servos {
  int pin_servo_m;
  int min_servo;
  int max_servo;

 public:
  Servos(int pin, int servo_sinal_min, int servo_sinal_max){
  pin_servo_m = pin;
  min_servo = servo_sinal_min;
  max_servo = servo_sinal_max;
  pinMode(pin_servo_m, OUTPUT); 
  }

  void colocar_angulo(int angulo_servo) {
    float pulseWidth = map(angulo_servo, 0, 180, min_servo, max_servo);  
    digitalWrite(pin_servo_m, HIGH);          
    delayMicroseconds(pulseWidth);        
    digitalWrite(pin_servo_m, LOW);
    delay(15);            
  }

};
Servos servo(PIN_SERVO, SERVO_SINAL_MIN, SERVO_SINAL_MAX );
#endif // EXIST_MOTOR_DC 

//-----------------------------------------------------------------------------
#if EXIST_Ultrassonico
//classe para controle dos sensores ultrassonicos
class Sensor_ultrassonico {
  int pin_echo;
  int pin_trig;
  float tempoEcho;
  float dist;  
  
 public:
  Sensor_ultrassonico(int echo, int trig){
  dist = 0;
  tempoEcho = 0;
  pin_echo = echo;
  pin_trig = trig;
  pinMode(pin_echo, INPUT); 
  pinMode(pin_trig, OUTPUT); 
  }

 float Calcula_dist(){
   noInterrupts();
   digitalWrite(pin_trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(pin_trig, LOW);
   tempoEcho = pulseIn(pin_echo, HIGH);
   dist = ((tempoEcho*velocidadeSom)/2)*100;
   interrupts();
   return dist;
 }

 bool Detectar_obstaculo(float distancia){
  if (distancia < DISTANCIA_PARAR){
    detec = 1;
    return true;
  }else{
    detec = 0;
    return false;
  }

 } 
};
Sensor_ultrassonico HCSR04_1(PIN_ECHO_1, PIN_TRIG_1);
#if EXIST_MEGA 
Sensor_ultrassonico HCSR04_2(PIN_ECHO_2, PIN_TRIG_2);
Sensor_ultrassonico HCSR04_3(PIN_ECHO_3, PIN_TRIG_3);
#endif // EXIST_MEGA 
#endif // EXIST_Ultrassonico

//-----------------------------------------------------------------------------
// classe responsavel por aplicar os filtros
#if EXIST_FILTRO
class Filtro {
  float** sinal;
  int intervalo_media_m;
  int numero_filtros;
  
 public:
  Filtro(int intervalo_media, int numero){
    intervalo_media_m = intervalo_media;
    numero_filtros = numero;
    sinal = new float*[numero_filtros];
    for (int i = 0; i < numero_filtros; i++) {
      sinal[i] = new float[intervalo_media_m];
      memset(sinal[i], 0, sizeof(float) * intervalo_media_m);
    }
  }

  ~Filtro() {
    for (int i = 0; i < numero_filtros; i++){delete[] sinal[i];}
    delete[] sinal;
   }

  float Media_movel(float sinal_) {
    float k = 0;
    for (int i = 0; i < numero_filtros; i++) {
      for (int x = intervalo_media_m - 1; x > 0; x--){sinal[i][x] = sinal[i][x - 1];}
      sinal[i][0] = sinal_;
      k = 0;
      for (int x = 0; x < intervalo_media_m; x++){k += sinal[i][x];}
    }
    return k / intervalo_media_m;
  }  
};
#if EXIST_VISAO_FILTRO
Filtro Filtro_visao(INTERVALO_MEDIA_VISAO, NUMERO_FILTROS_VISAO);
#endif // EXIST_VISAO_FILTRO
#if EXIST_Ultrassonico_FILTRO
Filtro Filtro_HCSR04_1(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
#if EXIST_MEGA 
Filtro Filtro_HCSR04_2(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
Filtro Filtro_HCSR04_3(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
#endif // EXIST_MEGA 
#endif // EXIST_Ultrassonico_FILTRO
#endif // EXIST_FILTRO

//-----------------------------------------------------------------------------
// classe para controle dos enconders
#if EXIST_ENCODER
class Encoder {
 public:
  int encoderPinA;
  int encoderPinB;
  volatile long contadorVoltas;
  volatile int encoderPosAnterior;
  bool trava_tempo_V;
  double velocidade_real; 
  double volta_inicial;
  double volta_final;
  double dist_per;
  unsigned long tempo_milis;
  unsigned long tempo_inicial;
  double tempo_passado;

  Encoder(int pinA, int pinB) : encoderPinA(pinA), encoderPinB(pinB), contadorVoltas(0), encoderPosAnterior(0) {}

  void setup() {
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    trava_tempo_V = true;
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderInterruptHandler, CHANGE);
  }

  double velocidade() {
    if(trava_tempo_V){
      tempo_inicial = micros();
      trava_tempo_V = false;
      volta_inicial = contadorVoltas / 2880.0;
      return velocidade_real;
    }else{
      tempo_passado = micros() - tempo_inicial;
      if(tempo_passado >= 500){
        noInterrupts();
        trava_tempo_V = true;
        tempo_passado = tempo_passado / 1000000;
        volta_final = contadorVoltas / 2880.0;
        velocidade_real = volta_final - volta_inicial;
        velocidade_real = velocidade_real / tempo_passado;
        velocidade_real = velocidade_real * 2 * 3.141592 * RAIO_RODA;
        interrupts();
        return velocidade_real;
      }else{return velocidade_real;}
    }
  }

  double dist_percorrida(){
    dist_per = contadorVoltas / 1440.0;
    #if EXIST_MOTOR_DC
    dist_per = contadorVoltas / 2880.0;
    #endif
    dist_per = dist_per * 2 * 3.141592 * RAIO_RODA;
    return dist_per;
  }

  static void encoderInterruptHandler() {
    if (instance) {
      instance->encoderInterrupt();
    }
  }
  static Encoder* instance;

 private:
  void encoderInterrupt() {
    int encoderA = digitalRead(encoderPinA);
    int encoderB = digitalRead(encoderPinB);
    int encoderPos = (encoderA << 1) | encoderB;

    if (encoderPos != encoderPosAnterior) {
      if ((encoderPosAnterior == 0b00 && encoderPos == 0b01) ||
          (encoderPosAnterior == 0b01 && encoderPos == 0b11) ||
          (encoderPosAnterior == 0b11 && encoderPos == 0b10) ||
          (encoderPosAnterior == 0b10 && encoderPos == 0b00)) {
        contadorVoltas++;
      } else if ((encoderPosAnterior == 0b00 && encoderPos == 0b10) ||
                 (encoderPosAnterior == 0b10 && encoderPos == 0b11) ||
                 (encoderPosAnterior == 0b11 && encoderPos == 0b01) ||
                 (encoderPosAnterior == 0b01 && encoderPos == 0b00)) {
        contadorVoltas--;
      }
    }

    encoderPosAnterior = encoderPos;
  }
};
Encoder* Encoder::instance = nullptr;
Encoder encoder_D(PIN_EN_DA, PIN_EN_DB);
#if EXIST_MEGA 
// Encoder encoder_E(PIN_EN_EA, PIN_EN_EB);
#endif // EXIST_MEGA 

//-----------------------------------------------------------------------------
// classe para controle dos enconders
class EncoderB {
 public:
  int encoderPinA;
  int encoderPinB;
  volatile long contadorVoltas;
  volatile int encoderPosAnterior;
  bool trava_tempo_B;
  double velocidade_real; 
  double volta_inicial;
  double volta_final;
  double dist_per;
  unsigned long tempo_milis;
  unsigned long tempo_inicial;
  double tempo_passado;

  EncoderB(int pinA, int pinB) : encoderPinA(pinA), encoderPinB(pinB), contadorVoltas(0), encoderPosAnterior(0) {}

  void setup() {
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    trava_tempo_B = true;
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterruptHandlerB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderInterruptHandlerB, CHANGE);
  }

  double velocidade() {
    if(trava_tempo_B){
      tempo_inicial = micros();
      trava_tempo_B = false;
      volta_inicial = contadorVoltas / 2880.0;
      return velocidade_real;
    }else{
      tempo_passado = micros() - tempo_inicial;
      if(tempo_passado >= 500){
        noInterrupts();
        trava_tempo_B = true;
        tempo_passado = tempo_passado / 1000000;
        volta_final = contadorVoltas / 2880.0;
        velocidade_real = volta_final - volta_inicial;
        velocidade_real = velocidade_real / tempo_passado;
        velocidade_real = velocidade_real * 2 * 3.141592 * RAIO_RODA;
        interrupts();
        return velocidade_real;
      }else{return velocidade_real;}
    }
  }

  double dist_percorrida(){
    dist_per = contadorVoltas / 1440.0;
    #if EXIST_MOTOR_DC
    dist_per = contadorVoltas / 2880.0;
    #endif
    dist_per = dist_per * 2 * 3.141592 * RAIO_RODA;
    return dist_per;
  }

  static void encoderInterruptHandlerB() {
    if (instance) {
      instance->encoderInterrupt();
    }
  }
  static EncoderB* instance;

 private:
  void encoderInterrupt() {
    int encoderA = digitalRead(encoderPinA);
    int encoderB = digitalRead(encoderPinB);
    int encoderPos = (encoderA << 1) | encoderB;

    if (encoderPos != encoderPosAnterior) {
      if ((encoderPosAnterior == 0b00 && encoderPos == 0b01) ||
          (encoderPosAnterior == 0b01 && encoderPos == 0b11) ||
          (encoderPosAnterior == 0b11 && encoderPos == 0b10) ||
          (encoderPosAnterior == 0b10 && encoderPos == 0b00)) {
        contadorVoltas++;
      } else if ((encoderPosAnterior == 0b00 && encoderPos == 0b10) ||
                 (encoderPosAnterior == 0b10 && encoderPos == 0b11) ||
                 (encoderPosAnterior == 0b11 && encoderPos == 0b01) ||
                 (encoderPosAnterior == 0b01 && encoderPos == 0b00)) {
        contadorVoltas--;
      }
    }

    encoderPosAnterior = encoderPos;
  }
};
EncoderB* EncoderB::instance = nullptr;
EncoderB encoder_E(PIN_EN_EA, PIN_EN_EB);
#endif // EXIST_ENCODER 

/*******************************************************************/
void setup() {

  Serial.begin(115200); // inicializa o monitor serial
  
  #if EXIST_ENCODER
  Encoder::instance = &encoder_D; 
  encoder_D.setup(); // inicializa o encoder direito
  #if EXIST_MEGA 
  EncoderB::instance = &encoder_E; 
  encoder_E.setup(); // inicializa o encoder esquerdo
  #endif // EXIST_MEGA 
  #endif // EXIST_ENCODER

  #if EXIST_BLUE
  HC06.begin(9600); // inicializa o modulo bluetooth HC06
  #endif // EXIST_BLUE

}

//-----------------------------------------------------------------------------
void loop() {
  
  #if EXIST_VISAO
  Visao_computacional();
  #endif // EXIST_VISAO

  #if EXIST_Ultrassonico
  Distancia_Sensor();
  #endif // EXIST_Ultrassonico
    
  #if EXIST_ENCODER
  Encoder_call();
  #endif // EXIST_ENCODER

  switch (switch_case) {
    case 1:
    #if EXIST_VISAO
    Autonomo();
    #endif //EXIST_VISAO
    break;

    case 2:
      #if EXIST_CALIBRA_PWM_MANUAL
      Ajuste_pwm_manual();
      #endif // EXIST_CALIBRA_PWM_MANUAL
    break;

    case 3:
      #if EXIST_CALIBRA_SERVO
      Ajuste_servo_manual();
      #endif // EXIST_CALIBRA_SERVO
    break;

    case 4:
      #if EXIST_CONTROLE_REMOTO
      Controle_remoto();
      #endif // EXIST_CONTROLE_REMOTO
    break;
  
    default:
      #if EXIST_MOTOR_DC 
      motor_direito.para();
      motor_esquerdo.para();
      #endif // EXIST_MOTOR_DC 
      dado_menu = "0";
      #if EXIST_BLUE
      if (HC06.available()) {
        msg_blue = HC06.read();
        #if EXIST_VISAO
        if(msg_blue == '1'){switch_case = 1;dado_menu = "1";HC06.println("Modo autonomo");}
        #endif //EXIST_VISAO
        #if EXIST_CALIBRA_PWM_MANUAL
        if(msg_blue == '2'){switch_case = 2;dado_menu = "2";HC06.println("Ajuste pwm manual");}
        #endif // EXIST_CALIBRA_PWM_MANUAL
        #if EXIST_CALIBRA_SERVO
        if(msg_blue == '3'){switch_case = 3;dado_menu = "3";HC06.println("Ajuste servo");}
        #endif // EXIST_CALIBRA_SERVO
        #if EXIST_CONTROLE_REMOTO
        if(msg_blue == '4'){switch_case = 4;dado_menu = "4";HC06.println("Controle remoto");}
        #endif // EXIST_CONTROLE_REMOTO 
        msg_blue = 0;       
      }
      #endif // EXIST_BLUE   
    break;
  }
  Prints();
}
/*******************************************************************/






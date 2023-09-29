#include <PID_v1.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//*************************************************************************
//Definindo existencia: 
//(1 = Existe,  0 = Não existe). 
//caso "não exista", toda a parte relacionada a essa existencia será comentada,
// de forma a não aparecer no monitor serial e nem pesar no processamento do arduino. 

#define EXIST_DADOS 1 // existencia de dados para print 
#define EXIST_BLUETOOTH 1 // existencia de filtros

#define EXIST_FILTRO 1 // existencia de filtros

#define EXIST_PID 1     // existencia de PID 
#define EXIST_PID_VEL (EXIST_PID && 1)
#define EXIST_NPID_VEL (!EXIST_PID_VEL)
#define EXIST_PID_OFFSET (EXIST_PID && 1)
#define EXIST_NPID_OFFSET (!EXIST_PID_OFFSET && 0)

#define EXIST_VISAO 1 // existencia do modulo bluetooth HC06
#define EXIST_VISAO_FILTRO (EXIST_FILTRO && EXIST_VISAO && 1)    // existencia de filtro nos dados do angulo da visão computacional
#define EXIST_OFFSET_FILTRO (EXIST_FILTRO && EXIST_VISAO && 1)    //existencia de filtro nos dados do offet da visão computacional
#define EXIST_VISAO_DADOS (EXIST_VISAO && 0)    // existencia dos dados da visão para print
#define EXIST_VISAO_ANGULO_DADOS (EXIST_VISAO_DADOS && 1)    // existencia dos dados da visão para print
#define EXIST_OFFSET_DADOS (EXIST_VISAO_DADOS && 1)    // existencia dos dados da visão para print

#define EXIST_MOTOR_DC_DADOS 0 // existencia dos motores dc PWM

#define EXIST_ENCODER 1 // existencia dos enconders
#define EXIST_ENCODER_DADOS 0   // existencia dos dados do encoder para print
#define EXIST_ENCODER_FILTRO (EXIST_FILTRO && EXIST_ENCODER && 1) 

#define EXIST_MPU6050 0 //define a existencia do MPU6050
#define EXIST_GYRO_DADOS (EXIST_MPU6050 && 1)  // existencia dos dados do giroscopio para print
#define EXIST_GYRO_FILTRO (EXIST_FILTRO && 1) //define a existencia de foltro do giroscopio

#define EXIST_SERVO_DADOS 0 // existencia do servo motoror

#define EXIST_ULTRA 0  // existencia do sensor ultrassonico
#define EXIST_ULTRA_FILTRO (EXIST_FILTRO && EXIST_ULTRA && 1) // existencia do filtro para o sensor ultrassonico
#define EXIST_ULTRA_MEIO (EXIST_ULTRA && 1)  // existencia do sensor ultrassonico do meio
#define EXIST_ULTRA_DIREITA (EXIST_ULTRA && 0)  // existencia do sensor ultrassonico da direita
#define EXIST_ULTRA_ESQUERDA (EXIST_ULTRA && 0)  // existencia do sensor ultrassonico da esquerda
#define EXIST_ULTRA_DADOS (EXIST_ULTRA && 1)  // existencia dos dados do sensor ultrassonico para print

#define EXIST_INFRA 0 // existencia do sensor infravermelho seguidr de linha
#define EXIST_INFRA_DADOS (EXIST_INFRA && 1)

#define EXIST_SWITCH_DADOS (EXIST_DADOS && 1)   // existencia dos dados do menu para print
#define EXIST_AJUSTE_GRAFICO (EXIST_DADOS && 0)


//------------------------------------------------------------------------------
// DEFININDO PINOS DO ARDUINO:

// Pinos do motor:
#define PIN_MD1 10   // pino 1 de controle do motor direito (dominante)
#define PIN_MD2 9   // pino 2 de controle do motor direito
#define PIN_ME1 12     // pino 1 de controle do motor esquerdo (dominante)
#define PIN_ME2 11       // pino 2 de controle do motor esquerdo

// Pinos dos encoders:
#define PIN_EN_DA 18     // pino A de controle do enconder dieito para arduino MEGA 18
#define PIN_EN_DB 19     // pino B de controle do enconder dieito para arduino MEGA 19
#define PIN_EN_EA 2     // pino A de controle do enconder esquerdo para arduino MEGA
#define PIN_EN_EB 3    // pino B de controle do enconder esquerdo para arduino MEGA

// Pino do servo:
#define PIN_SERVO 8    // pino de controle do servo motor 8

// Pinos dos sensores ultrassonicos
#define PIN_TRIG_1 7     // pino trig do ultrassonico 1 meio
#define PIN_ECHO_1 6     // pino echo do ultrassonico 1 meio
#define PIN_TRIG_2 26     // pino trig do ultrassonico 2 direita
#define PIN_ECHO_2 25     // pino echo do ultrassonico 2 direita
#define PIN_TRIG_3 28     // pino trig do ultrassonico 3 esquerda
#define PIN_ECHO_3 27     // pino echo do ultrassonico 3 esquerda

// Pinos dos sensores infravermelhos
#define PIN_INFRA_D 46     // pinos do sensor infra direito
#define PIN_INFRA_E 30     // pinos do sensor infra esquerdo

// Pinos do modulo bluetooth
SoftwareSerial HC06(50, 51); // pinos TX, RX do bluetooth para arduino MEGA

//-----------------------------------------------------------------------------
// DEFININDO VALORES CONSTANTES:

//Sobre os motores:
#define PWM_MAXIMO 140     // pwm maximo para fazer o motor girar (0 a 225)
#define PWM_MINIMO 80     // pwm minimo para fazer o motor girar (0 a 225)

//Sobre os encoders:
#define VEL_MAX 0.3  // velocidade maxima (m/s) que o carro deve atingir 
#define RAIO_RODA 0.175     // raio da roda em metros
#define NUM_PULSO_VOLTA 2880.0     // numero de opulsos necessarios para o enconder contabilizar 1 volta 1440.0 = 1 volta | 2880.0 = 2 voltas

#define TIME_FRENAGEM_FOFO 0.08    // intervalo de tempo para alterar o pwm durante a frenagem (em milisegundos)
#define TIME_ACELERA_FOFO 0.1     // intervalo de tempo para alterar o pwm durante a aceleraçao de arranque do carro (em milisegundos)
#define TIME_OFFSET 10     // intervalo de tempo para alterar o angulo do offset 

//-----filtro do encoder-----:
#define INTERVALO_MEDIA_ENCODER 50   // numero de valores para efetuar a media
#define NUMERO_FILTROS_ENCODER 1     // numero de filtros que será aplicado

//PID cruzeiro:
#define KP_MC 400  //bom = 180   400
#define KI_MC 400 //bom = 110    400
#define KD_MC 20  //bom = 5    20

//PID curava:
#define KP_MCU 400  //bom = 
#define KI_MCU 500  //bom = 
#define KD_MCU 20   //bom = 

//PID frenagem fofa:
#define KP_MFF 400   //bom = 
#define KI_MFF 950   //bom = 
#define KD_MFF 5     //bom = 

//PID OFFSET:
#define KP_OFF 0.6    // bom = 0.2
#define KI_OFF 0.9   // bom =  0.3  
#define KD_OFF 0     //0
#define TEMPO_PID_OFFSET 200  

//Sobre os servos:
#define ANGULO_INICIAL 80   // angulo real inicial do servo para quando ligar o carro
#define ANGULO_ZERO 80  // angulo real que sera considerado o ponto zero (deixar as rodas retas) 
#define ANGULO_MAX 150   // angulo real maximo que o servo pode atingir 
#define ANGULO_MIN 10    // angulo minimo real que o servo pode atingir (0)
#define SERVO_SINAL_MIN 500      // sinal em microsegundos do angulo minimo do servo (configuração do servo) 500
#define SERVO_SINAL_MAX 2400      // sinal em microsegundos do angulo maximo do servo (configuração do servo)

//Sobre os sensores ultrassonicos:
#define DISTANCIA_PARAR 75   // distancia minima (em cm) para o carro parar
#define DISTANCIA_DETECTA 75     // distancia minima (em cm) para detectar a presença de um corpo 100

//-----filtro do sensor ultrassonico-----:
#define INTERVALO_MEDIA_HCSR04 10    // numero de valores para efetuar a media
#define NUMERO_FILTROS_HCSR04 1     // numero de filtros que será aplicado

//Sobre a visão computacional:
#define FATOR_ANGULO_VISAO 3   //numero que será multiplicado ao angulo de visão recebido
//-----filtro da visão compitacional -----:
#define INTERVALO_MEDIA_VISAO 10     // numero de valores para efetuar a media
#define NUMERO_FILTROS_VISAO 1    // numero de filtros que será aplicado

#define INTERVALO_MEDIA_OFFSET 10     // numero de valores para efetuar a media
#define NUMERO_FILTROS_OFFSET 1    // numero de filtros que será aplicado

//Sobre o MPU6050:
#define MEDIA_PARA_GIRO 3000      // media para tarar os angulos do giroscopio
#define MEDIA_OFFSET 25      // media para impedir o almento constante do angulo parado
#define MEDIA_OFFSET_Z 40     // media para impedir o almento constante do angulo parado

//-----filtro dO mp6050-----:
#define INTERVALO_MEDIA_GIRO 20
#define NUMERO_FILTROS_GIRO 1

#define INICIO_DA_CURVA 7
#define INICIO_DA_CURVA_NEG -7

//-----------------------------------------------------------------------------
// VARIAVEIS GLOBAIS

int switch_case = 1;    // variavel que controla os casos do switch case do menu 
int auto_estado = 3;    // variavel que controla os casos do switch case do modo autonomo
int remoto_estado = 1;  // variavel que controla os casos do switch case do modo de controle remoto

int estado_motor; // indica por meio de 0 ou 1 se o motor está ligado ou desligado
int agr_vai;

double zero = 0;
double pwm = PWM_MAXIMO;
double pwm_d = 0; // pwm inicial
double pwm_e = 0; // pwm inicial
double pwm_min = PWM_MINIMO; // pwm maximo que o carro irá atingir
double pwm_max = PWM_MAXIMO; // pwm minimo que o carro precisa para andar

double kp_mc = KP_MC;
double ki_mc = KI_MC;
double kd_mc = KD_MC;

double kp_me = KP_MC;
double ki_me = KI_MC;
double kd_me = KD_MC;

double kp_off = KP_OFF;
double ki_off = KI_OFF;
double kd_off = KD_OFF;

double angulo_z_f, angulo_z_f_ant;
double angulo_x_f;

int offset_teste;


int angulo_servo = ANGULO_INICIAL; // armazena o angulo real do servo motor 
int angulo_zero = ANGULO_ZERO; // armazena apenas o angulo que irá definir o ponto zero 
int angulo_maximo = ANGULO_MAX; // armazena o angulo real maximo que o servo consegue abrir
int angulo_minimo = ANGULO_MIN; // armazena o angulo real maximo que o servo consegue abrir
int intervalo_maximo = angulo_maximo - angulo_zero;
int intervalo_minimo = angulo_minimo - angulo_zero;

int angulo_teste, angulo_teste_2, trava_reto;

int detec_meio;
int detec_direita;
int detec_esquerda;

#if EXIST_VISAO
int angulo_visao, angulo_visao_real, angulo_visao_f, angulo_visao_antigo; // armazena o angulo dado pela visão computacional
int esquerda, direita;
int offset;
double offset_double, offset_antigo;
double angulo_offset;
#endif //EXIST_VISAO

bool obstaculo = false; // armazena a indicação de obstaculo no caminho do sensor 3
bool obstaculo_1 = false; // armazena a indicação de obstaculo no caminho do sensor 1
bool obstaculo_2 = false; // armazena a indicação de obstaculo no caminho do sensor 2
bool obstaculo_3 = false; // armazena a indicação de obstaculo no caminho do sensor 3
bool dist_grande = false;
bool trava_gyro = false;  // trava para offset do gyroscopio
bool trava_chao = true; // trava para offset do gyroscopio
bool trava_pid_vel = false;
bool trava_pid_offset = false;
bool trava_ultrasson = false;

bool trava_teste_reta = false;
bool trava_curva_saida = true;

bool detec_curva = false;

int dado_infra;

double vel_max = VEL_MAX;
double vel_max_d = VEL_MAX;
double vel_max_e = VEL_MAX;
double vel_md, vel_md_f, vel_me, vel_me_f; // armazenam a velocidade em (m/s) dos motores direito e esquerdo respectivamente
double dist_total; // armazenam a distancia total percorrida

String dados_print_HC06 = " ";  // armazena os dados que serão printado no bluetooth
String dados_print_PC = " ";  // armazena os dados que serão printado no monitor serial
String dado_menu = "0"; // armazena o estado do switch case
String dados_visao = " "; // armazena os dado oriondos da visão computacional


char msg_blue; // armazena os dados recebido do bluetooth ou monitor serial do pc 


#if EXIST_ULTRA
const float velocidadeSom = 0.00034029; // velocidade do som em metros/microsegundo
float distancia_1, distancia_1f, distancia_2, distancia_2f, distancia_3, distancia_3f;
int detec;
#endif // EXIST_ULTRA

#if EXIST_MPU6050
double gyroX; //armazenam os angulos brutos do giroscopio no eixo x
double gyroXoffset; //armazena o valor excedente de referencia do giroscopio no eixo x
double angleX, angulo_x; //armazenam os angulos do giroscopio depois de tratado no eixo x
int cont_offsetX, angulo_x_set, angulo_x_setoff;
int cont_offsetZ, angulo_z_set, angulo_z_setoff;
double gyroZ; //armazenam os angulos brutos do giroscopio no eixo Z
double gyroZoffset; //armazena o valor excedente de referencia do giroscopio no eixo Z
double angleZ, angulo_z; //armazenam os angulos do giroscopio depois de tratado no eixo Z
float interval; //armazena a variação do angulo em um determinado tempo 
long preInterval; // tempo de variação do angulo do giroscopio 
#endif //EXIST_MPU6050


//*****************************************************************************
//BIBLIOTECAS E CLASSES:

#if EXIST_MPU6050
//biblioteca do MPU6050:
const uint8_t IMUAddress = 0x68;
const uint8_t I2C_TIMEOUT = 1000;
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop){
  return i2cWrite(registerAddress, &data, 1, sendStop);
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if(rcode){
    Serial.print(F("i2cWrite Failed: "));
    Serial.println(rcode);
  }
  return rcode;
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes){
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if(rcode){
    Serial.print(F("i2cRead Failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for(uint8_t i = 0; i< nbytes; i++){
    if(Wire.available())
      data[i]=Wire.read();
    else{
      timeOutTimer = micros();
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if(Wire.available())
        data[i] = Wire.read();
      else{
        Serial.println(F("i2cRead timeout"));
        return 5;
      }
    }
  }
  return 0;
}
uint8_t i2c_data[14]; //configuração do mpu6050
#endif //EXIST_MPU6050

//------------------------------------------------------------------------------
// Classe responsavel pelo setup e orientação dos motores:
class Motores {
  int pin_m1;
  int pin_m2;
  unsigned long intervalo;
  unsigned long ultima_atualizacao;
  bool trava_chamado;

 public:
  Motores(int pin_1, int pin_2){
    pin_m1 = pin_1;
    pin_m2 = pin_2;
    pinMode(pin_m1, OUTPUT);  
    pinMode(pin_m2, OUTPUT);
    intervalo = 100;
    trava_chamado = true;
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

//-----------------------------------------------------------------------------
// Classe responsavel pela contagem de tempo em millis():
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
Contador_tempo time_frenagem_fofo_d(TIME_FRENAGEM_FOFO);
Contador_tempo time_frenagem_fofo_e(TIME_FRENAGEM_FOFO);
Contador_tempo time_acelera_fofo_d(TIME_ACELERA_FOFO);
Contador_tempo time_acelera_fofo_e(TIME_ACELERA_FOFO);
Contador_tempo time_offset(TIME_OFFSET);
Contador_tempo time_print(10); //itervalo de tempo para printar
Contador_tempo time_fofo(1000); 
Contador_tempo time_servo(100);
Contador_tempo time_curva_e(7500); //7500
Contador_tempo time_curva_d(9500);



//-----------------------------------------------------------------------------
//Classe para controle de servos
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

//-----------------------------------------------------------------------------
#if EXIST_ULTRA
//Classe para controle dos sensores ultrassonicos
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

  unsigned long timeout = 20000;  // Tempo máximo de espera
  tempoEcho = pulseIn(pin_echo, HIGH, timeout); // Mede o tempo de eco com timeout
  if(tempoEcho == 0) {
    // Se pulseIn atingir o timeout, defina a distância como um valor grande (por exemplo, 9999)
    dist_grande = true;
  }else{
    dist = (tempoEcho * velocidadeSom) / 2 * 100;
    dist_grande = false;
  }

  //  tempoEcho = pulseIn(pin_echo, HIGH);
  //  dist = ((tempoEcho*velocidadeSom)/2)*100;
   interrupts();
   return dist;
 }

 bool Detectar_obstaculo(float distancia){
  if (distancia < DISTANCIA_DETECTA){
    return true;
  }else{
    return false;
  }

 } 
};
Sensor_ultrassonico HCSR04_1(PIN_ECHO_1, PIN_TRIG_1);
Sensor_ultrassonico HCSR04_2(PIN_ECHO_2, PIN_TRIG_2);
Sensor_ultrassonico HCSR04_3(PIN_ECHO_3, PIN_TRIG_3);
#endif // EXIST_ULTRA

//-----------------------------------------------------------------------------
#if EXIST_INFRA
// Classe responsavel pelo setup e orientação dos motores:
class Infravermelho {
  int pin_infra;

 public:
  Infravermelho(int pininfra){
    pin_infra = pininfra;
    pinMode(pin_infra, INPUT);    
  }

  bool Detectar_linha() {
    if (digitalRead(pin_infra) == HIGH){
      dado_infra = 0;
      return false;
    }else{
      dado_infra = 1;
      return true;
    }
  }
};
Infravermelho infra_direito(PIN_INFRA_D);
Infravermelho infra_esquerdo(PIN_INFRA_E);
#endif // EXIST_INFRA

//-----------------------------------------------------------------------------
// Classe responsavel por aplicar os filtros
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
    noInterrupts();
    for (int i = 0; i < numero_filtros; i++) {
      sinal[i] = new float[intervalo_media_m];
      memset(sinal[i], 0, sizeof(float) * intervalo_media_m);
    }
    interrupts();
  }

  ~Filtro() {
    noInterrupts();
    for (int i = 0; i < numero_filtros; i++){delete[] sinal[i];}
    interrupts();
    delete[] sinal;
   }

  float Media_movel(float sinal_) {
    float k = 0;
    noInterrupts();
    for (int i = 0; i < numero_filtros; i++) {
      for (int x = intervalo_media_m - 1; x > 0; x--){sinal[i][x] = sinal[i][x - 1];}
      sinal[i][0] = sinal_;
      k = 0;
      for (int x = 0; x < intervalo_media_m; x++){k += sinal[i][x];}
    }
    interrupts();
    return k / intervalo_media_m;
  }  
};
#if EXIST_GYRO_FILTRO
Filtro Filtro_giro_x(INTERVALO_MEDIA_GIRO, NUMERO_FILTROS_GIRO);
Filtro Filtro_giro_z(INTERVALO_MEDIA_GIRO, NUMERO_FILTROS_GIRO);
#endif

#if EXIST_ENCODER_FILTRO
Filtro Filtro_VEL_D(INTERVALO_MEDIA_ENCODER, NUMERO_FILTROS_ENCODER);
Filtro Filtro_VEL_E(INTERVALO_MEDIA_ENCODER, NUMERO_FILTROS_ENCODER);
#endif // EXIST_ENCODER_FILTRO

#if EXIST_VISAO_FILTRO
Filtro Filtro_visao(INTERVALO_MEDIA_VISAO, NUMERO_FILTROS_VISAO);
#endif // EXIST_VISAO_FILTRO
#if EXIST_OFFSET_FILTRO
Filtro Filtro_offset(INTERVALO_MEDIA_OFFSET, NUMERO_FILTROS_OFFSET);
#endif // EXIST_OFFSET_FILTRO

#if EXIST_ULTRA_FILTRO
Filtro Filtro_HCSR04_1(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
Filtro Filtro_HCSR04_2(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
Filtro Filtro_HCSR04_3(INTERVALO_MEDIA_HCSR04, NUMERO_FILTROS_HCSR04);
#endif // EXIST_ULTRA_FILTRO
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
      volta_inicial = contadorVoltas / NUM_PULSO_VOLTA;
      return velocidade_real;
    }else{
      tempo_passado = micros() - tempo_inicial;
      if(tempo_passado >= 1000){
        noInterrupts();
        trava_tempo_V = true;
        tempo_passado = tempo_passado / 1000000;
        volta_final = contadorVoltas / NUM_PULSO_VOLTA;
        velocidade_real = volta_final - volta_inicial;
        velocidade_real = velocidade_real / tempo_passado;
        velocidade_real = velocidade_real * 2 * 3.141592 * RAIO_RODA;
        interrupts();
        return velocidade_real;
      }else{return velocidade_real;}
    }
  }

  double dist_percorrida(){
    dist_per = contadorVoltas / NUM_PULSO_VOLTA;
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
      volta_inicial = contadorVoltas / NUM_PULSO_VOLTA;
      return velocidade_real;
    }else{
      tempo_passado = micros() - tempo_inicial;
      if(tempo_passado >= 1000){
        noInterrupts();
        trava_tempo_B = true;
        tempo_passado = tempo_passado / 1000000;
        volta_final = contadorVoltas / NUM_PULSO_VOLTA;
        velocidade_real = volta_final - volta_inicial;
        velocidade_real = velocidade_real / tempo_passado;
        velocidade_real = velocidade_real * 2 * 3.141592 * RAIO_RODA;
        interrupts();
        return velocidade_real;
      }else{return velocidade_real;}
    }
  }

  double dist_percorrida(){
    dist_per = contadorVoltas / NUM_PULSO_VOLTA;
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

//-----------------------------------------------------------------------------
// classe para PID
#if EXIST_PID_VEL 
PID PID_VEL_D_PWM(&vel_md_f, &pwm_d, &vel_max_d, kp_mc, ki_mc, kd_mc, P_ON_M, DIRECT); //Kp, Ki, Kd
PID PID_VEL_E_PWM(&vel_me_f, &pwm_e, &vel_max_e, kp_mc, ki_mc, kd_mc, P_ON_M, DIRECT); 
#endif //EXIST_PID_VEL 
#if EXIST_PID_OFFSET
PID PID_OFFSET(&offset_double, &angulo_offset, &zero, kp_off, ki_off, kd_off, P_ON_M, DIRECT); 
#endif //EXIST_PID_OFFSET


//***************************************************************************
void setup() {

  #if EXIST_BLUETOOTH
  switch_case = 0;  // variavel que controla os casos do switch case
  auto_estado = 0;
  remoto_estado = 0;
  #endif

  Serial.begin(9600); // inicializa o monitor serial
  Serial.setTimeout(100);
  Serial2.begin(57600); // inicializa a comunicação serial do lora
  HC06.begin(9600); // inicializa o modulo bluetooth HC06
  
  #if EXIST_ENCODER
  Encoder::instance = &encoder_D; 
  encoder_D.setup(); // inicializa o encoder direito 
  EncoderB::instance = &encoder_E; 
  encoder_E.setup(); // inicializa o encoder esquerdo
  #endif // EXIST_ENCODER

  #if EXIST_MPU6050
  Wire.begin();
  Wire.setClock(4000000UL);
  i2c_data[0] = 7;
  for(int i = 1; i<4; i++){i2c_data[i] = 0x00;}
  while(i2cWrite(0x19, i2c_data, 4, false));
  while(i2cWrite(0x6B, 0x01, true));
  while(i2cRead(0x75, i2c_data, 1));
  if(i2c_data[0]!= 0x68){
    Serial.print("Erro. Placa desconhecida!\n");
    while(1){Serial.println("Erro. Conecte a MPU6050 no barramento I2C.\n");}
  }
  delay(100);
  #endif // EXIST_MPU6050


  #if EXIST_PID_VEL 
  PID_VEL_D_PWM.SetSampleTime(50);
  PID_VEL_E_PWM.SetSampleTime(50);
  PID_VEL_D_PWM.SetOutputLimits(0, 255);
  PID_VEL_E_PWM.SetOutputLimits(0, 255);
  // PID_VEL_D_PWM.SetMode(AUTOMATIC);
  // PID_VEL_E_PWM.SetMode(AUTOMATIC);
  #endif //EXIST_PID_VEL

  #if EXIST_PID_OFFSET
  PID_OFFSET.SetOutputLimits(intervalo_minimo, intervalo_maximo);
  PID_OFFSET.SetSampleTime(TEMPO_PID_OFFSET); //150 valor que estava
  // PID_OFFSET.SetMode(AUTOMATIC);
  #endif //EXIST_PID_OFFSET

}

//-----------------------------------------------------------------------------
void loop() {
  
  #if EXIST_VISAO
  Visao_computacional();
  #endif // EXIST_VISAO

  #if EXIST_ULTRA
  Distancia_Sensor();
  #endif // EXIST_ULTRA
    
  #if EXIST_ENCODER
  Encoder_call();
  #endif // EXIST_ENCODER
  
  #if EXIST_MPU6050
  if(trava_gyro){Giroscopio();}
  #endif // EXIST_MPU6050
  
  #if EXIST_PID_VEL 
  if(trava_pid_vel){PID_VEL_D_PWM.Compute();PID_VEL_E_PWM.Compute();}
  #endif //EXIST_PID_VEL 

  #if EXIST_PID_OFFSET
  if(trava_pid_offset){PID_OFFSET.Compute();}
  #endif //EXIST_PID_OFFSET

  switch (switch_case) {
    case 1:
    #if EXIST_VISAO
    Autonomo();
    #endif //EXIST_VISAO
    break;

    case 2:
      Ajuste_pwm_manual();
    break;

    case 3:
      Ajuste_servo_manual();
    break;

    case 4:
      Controle_remoto();
    break;
  
    default:
      motor_direito.para();
      motor_esquerdo.para(); 
      dado_menu = "0";

      if (HC06.available()) {
        msg_blue = HC06.read();

        #if EXIST_VISAO
        if(msg_blue == '1'){switch_case = 1;}//autonomo;}
        #endif //EXIST_VISAO

        if(msg_blue == '2'){switch_case = 2;}//pwm ajuste}

        if(msg_blue == '3'){switch_case = 3;}//servo ajuste;}

        if(msg_blue == '4'){switch_case = 4;} //controle remoto}

        msg_blue = 0;     
      }  
    break;
  }

  Prints();
}
//*****************************************************************************


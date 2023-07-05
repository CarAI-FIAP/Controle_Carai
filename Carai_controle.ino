#include <Wire.h>
#include <SoftwareSerial.h>
//-----------------------------------------------------------------------------
//Definindo existencia: 
//(1 = Existe,  0 = Não existe). 
//caso "não exista", toda a parte relacionada a essa existencia será comentada,
// de forma a não aparecer no monitor serial e nem pesar no processamento do arduino. 

#define EXIST_UNO 0 // por enqunto define se vai utilizar arduino UNO ou mega
#define EXIST_MEGA (!EXIST_UNO)

#define EXIST_BLUE 1 // existencia do modulo bluetooth 

#define EXIST_MOTOR_DC 1 // existencia dos motores dc
#define EXIST_ENCODER (EXIST_MOTOR_DC && 0) // existencia dos enconders
#define EXIST_CALIBRA_PWM (EXIST_ENCODER && 1) // existencia da função de calibrar o pwm minimo e maximo automatico
#define EXIST_CALIBRA_PWM_MANUAL (!EXIST_CALIBRA_PWM && EXIST_BLUE && EXIST_MOTOR_DC && 1) // existencia da função de calibrar o pwm minimo e maximo manualmente 

#define EXIST_SERVO 1 // existencia do servo motoror
#define EXIST_CALIBRA_SERVO (EXIST_SERVO && EXIST_BLUE && 1) // existencia da função de calibrar manualmente o servo motor

#define EXIST_CONTROLE_REMOTO (EXIST_BLUE && EXIST_MOTOR_DC && EXIST_SERVO && 1) // existencia de controlar o carro remotamente

#define EXIST_DADOS 1
#define EXIST_AJUSTE_GRAFICO (EXIST_DADOS && 0)

//-----------------------------------------------------------------------------
// Definindo pinos do arduino:
#define PIN_MD1 12 // pino 1 de controle do motor direito
#define PIN_MD2 11 // pino 2 de controle do motor direito
#define PIN_ME1 10 // pino 1 de controle do motor esquerdo
#define PIN_ME2 9  // pino 2 de controle do motor esquerdo

#define PIN_SERVO 8 // pino de controle do servo motor 

#if EXIST_UNO
#if EXIST_BLUE
SoftwareSerial HC06(6, 5); //define os pinos TX, RX do bluetooth para arduino MEGA
#endif
#endif
#if EXIST_MEGA 
#if EXIST_BLUE
SoftwareSerial HC06(50, 51); //define os pinos TX, RX do bluetooth para arduino MEGA
#endif
#endif

//-----------------------------------------------------------------------------
// Definindo constantes:
#define PWM_MAXIMO 225  // Pwm maximo para fazer o motor girar (0 a 225)
#define PWM_MINIMO 0  // Pwm minimo para fazer o motor girar (0 a 225)
#define VEL_MAX 6 // Velocidade maxima (m/s) que o carro deve atingir 

#define ANGULO_INICIAL 90 // angulo (graus) inicial do servo
#define SERVO_SINAL_MIN 500 //sinal em microsegundos do angulo minimo do servo
#define SERVO_SINAL_MAX 2400 //sinal em microsegundos do angulo maximo do servo

//-----------------------------------------------------------------------------
// variaveis globais:
int switch_case;  // variavel que controla os casos do switch case

int pwm = PWM_MAXIMO; //pwm inicial

int angulo_teorico = ANGULO_INICIAL; // recebe o angulo (via bluetooth) que servo deve atingir

String dados_print_HC06 = " ";  //armazena os dados que serão printado no bluetooth
String dados_print_PC = " ";  //armazena os dados que serão printado no monitor serial

#if EXIST_BLUE
char msg_blue; // armazena os dados recebido do bluetooth ou monitor serial do pc 
bool trava_menu_1 = true;
bool trava_menu_2 = true;
bool trava_menu_3 = true;
bool trava_menu_4 = true;
bool trava_menu_defull = true;
#endif

#if EXIST_SERVO
int angulo_servo = ANGULO_INICIAL; // angulo inicial do servo
#endif


#if EXIST_MOTOR_DC 
int pwm_min = PWM_MINIMO; //pwm maximo que o carro irá atingir
int pwm_max = PWM_MAXIMO; //pwm minimo que o carro precisa para andar
int estado_motor;
#endif

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
#endif

/*******************************************************************/
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
Contador_tempo ajuste_media_movel(); 
Contador_tempo parado(); 

/*******************************************************************/
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

  void controle_ajuste(int angulo_servo) {
    float pulseWidth = map(angulo_servo, 0, 180, min_servo, max_servo);  
    digitalWrite(pin_servo_m, HIGH);          
    delayMicroseconds(pulseWidth);        
    digitalWrite(pin_servo_m, LOW);
    delay(15);            
  }

};
Servos servo(PIN_SERVO, SERVO_SINAL_MIN, SERVO_SINAL_MAX );
#endif

/*******************************************************************/
void setup() {

  Serial.begin(115200);
  
  #if EXIST_BLUE
  HC06.begin(9600); 
  #endif
  
  #if EXIST_SERVO 
  servo.controle_ajuste(ANGULO_INICIAL); 
  #endif
  
}

/*******************************************************************/
void loop() {
  switch (switch_case) {
  case 1:
    //calibração automatica
    switch_case = 0;
  break;

  case 2:
    #if EXIST_CALIBRA_PWM_MANUAL
    Ajuste_pwm_manual();
    #endif
  break;

  case 3:
    #if EXIST_CALIBRA_SERVO
    Ajuste_servo_manual();
    #endif
  break;

  case 4:
    #if EXIST_CONTROLE_REMOTO
    Controle_remoto();
    #endif
  break;
  
  default:
    #if EXIST_MOTOR_DC 
    motor_direito.para();
    motor_esquerdo.para();
    #endif 
    
    #if EXIST_BLUE
    if (HC06.available()) {
      msg_blue = HC06.read();
      
      if(msg_blue == '1'){switch_case = 1;HC06.println("Ajuste pwm automatico");}
      
      #if EXIST_CALIBRA_PWM_MANUAL
      if(msg_blue == '2'){switch_case = 2;HC06.println("Ajuste pwm manual");}
      #endif
      #if EXIST_CALIBRA_SERVO
      if(msg_blue == '3'){switch_case = 3;HC06.println("Ajuste servo");}
      #endif
      #if EXIST_CONTROLE_REMOTO
      if(msg_blue == '4'){switch_case = 4;HC06.println("Controle remoto");}
      #endif
      msg_blue = 0;       
    }
    #endif    
  break;
  }
  Prints();
}

/*******************************************************************/
// definir o pwm minimo e maximo
#if EXIST_CALIBRA_PWM_MANUAL
void Ajuste_pwm_manual(){
  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'B'){
      #if EXIST_MOTOR_DC 
      pwm_min = pwm;
      #endif
      dados_print_HC06 += "Minimo ";
      dados_print_HC06 += "\t"; 
    }else if(msg_blue == 'A'){
      #if EXIST_MOTOR_DC 
      pwm_max = pwm;
      #endif 
      dados_print_HC06 += "Maximo ";
      dados_print_HC06 += "\t";      
    }else if(msg_blue == 'C'){
      dados_print_HC06 += "Calibração do motor finalizada ";
      dados_print_HC06 += "\t";
      #if EXIST_MOTOR_DC 
      motor_direito.para();
      motor_esquerdo.para();
      #endif
      switch_case = 0;
    }else if(msg_blue == '1'){
      pwm++;
      if(pwm > 255){pwm = 225;}
      msg_blue = 0;      
    }else if(msg_blue == '2'){
      pwm--;     
      if(pwm < 0){pwm = 0;}        
      msg_blue = 0;         
    }
    dados_print_HC06 += "PWM =: ";
    dados_print_HC06 += String(pwm);
    dados_print_HC06 += "\t"; 
    #if EXIST_MOTOR_DC      
    motor_direito.frente(pwm);
    motor_esquerdo.frente(pwm);   
    #endif  
  }
}
#endif

/*******************************************************************/
// Ajusta o movimento do servo
#if EXIST_CALIBRA_SERVO
void Ajuste_servo_manual(){
  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'C'){
      dados_print_HC06 += "Calibração do servo finalizada ";
      dados_print_HC06 += "\t";
      msg_blue = 0;
      switch_case = 0;
    }else if(msg_blue == '1'){
      angulo_teorico++;
      if(angulo_teorico > 180){angulo_teorico = 180;}
      msg_blue = 0;      
    }else if(msg_blue == '2'){
      angulo_teorico--;
      if(angulo_teorico < 0){angulo_teorico = 0;}        
      msg_blue = 0;         
    } 
  dados_print_HC06 += "angulo do servo = ";
  dados_print_HC06 += String(map(angulo_teorico, 0, 180, -90, 90));
  dados_print_HC06 += "\t";
  }
  #if EXIST_SERVO
  servo.controle_ajuste(angulo_teorico); 
  #endif
}
#endif

/*******************************************************************/
// controle remoto do veiculo
#if EXIST_CONTROLE_REMOTO
void Controle_remoto(){
 if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'A'){
      motor_direito.frente(pwm);
      motor_esquerdo.frente(pwm); 
    }else if(msg_blue == 'B'){
      motor_direito.para();
      motor_esquerdo.para();
    }else if(msg_blue == 'C'){
      motor_direito.para();
      motor_esquerdo.para();
      switch_case = 0;
    }else if(msg_blue == '1'){
      angulo_teorico++;
      if(angulo_teorico > 180){angulo_teorico = 180;}
      msg_blue = 0;
    }else if(msg_blue == '2'){
      angulo_teorico--;
      if(angulo_teorico < 0){angulo_teorico = 0;}
      msg_blue = 0;
    }
  }
  servo.controle_ajuste(angulo_teorico); 
}
#endif

/*******************************************************************/
void Prints(){
  
  #if EXIST_DADOS
  #if EXIST_MOTOR_DC 
  dados_print_PC += String(estado_motor);
  dados_print_PC += " ";
  #endif
  dados_print_PC += String(pwm);
  dados_print_PC += " ";
  // dados_print_PC += String(vel);
  // dados_print_PC += "\t";
  // dados_print_PC += String(dist_p);
  // dados_print_PC += "\t";
  #if EXIST_SERVO
  dados_print_PC += String(map(angulo_teorico, 0, 180, -90, 90));
  dados_print_PC += " ";
  #endif

  #if EXIST_AJUSTE_GRAFICO
  dados_print_PC += String(50);
  dados_print_PC += "\t";
  dados_print_PC += String(0);
  dados_print_PC += "\t";
  dados_print_PC += String(-50);
  dados_print_PC += "\t";
  #endif  
  Serial.println(dados_print_PC);
  #endif



  #if EXIST_BLUE
  if(dados_print_HC06 != " "){HC06.println(dados_print_HC06);}
  #endif
  
  dados_print_PC = " ";
  dados_print_HC06 = " ";
  
}













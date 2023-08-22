#if EXIST_VISAO
void Autonomo(){
  if (HC06.available()) {
    msg_blue = HC06.read();
   if(msg_blue == 'C'){
      #if EXIST_MOTOR_DC 
      motor_direito.para();
      motor_esquerdo.para();
      #endif // EXIST_MOTOR_DC 
      switch_case = 0;
    }
  }
  if(obstaculo){
    #if EXIST_MOTOR_DC 
    motor_direito.para();
    motor_esquerdo.para();
    #endif // EXIST_MOTOR_DC 
  }else{
    //ajuste_velocidade()
    #if EXIST_MOTOR_DC 
    motor_direito.frente(pwm);
    motor_esquerdo.frente(pwm); 
    #endif // EXIST_MOTOR_DC 
  }
  if(angulo_visao_real < ANGULO_MIN){
    angulo_visao_real = ANGULO_MIN;
  }else if(angulo_visao_real > ANGULO_MAX){
    angulo_visao_real = ANGULO_MAX;
  }
  servo.colocar_angulo(angulo_visao_real); 
}
#endif // EXIST_VISAO
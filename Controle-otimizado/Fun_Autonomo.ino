#if EXIST_VISAO
void Autonomo(){

  //Dividir em switch case **********>>>>>

  #if EXIST_MPU6050
  if(trava_chao){
    Call_ref_chao();
    trava_chao = false;
  }
  #endif

  if (HC06.available()) {
    msg_blue = HC06.read();
   if(msg_blue == 'C'){
      #if EXIST_MOTOR_DC 
      motor_direito.para();
      motor_esquerdo.para();
      #endif // EXIST_MOTOR_DC 
      trava_chao = true;
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
    motor_direito.frente(pwm_max);
    motor_esquerdo.frente(pwm_max); 
    #endif // EXIST_MOTOR_DC 
  }
  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  }

    
  servo.colocar_angulo(angulo_visao_real); 
}
#endif // EXIST_VISAO
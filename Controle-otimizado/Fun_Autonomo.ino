#if EXIST_VISAO
void Autonomo(){
  
  #if EXIST_MPU6050
  // faz a tara do mpu6050
  if(trava_chao){Call_ref_chao();trava_chao = false;}
  #endif //EXIST_MPU6050

  // freia fofo quando ultrassonico detecta um obstaculo
  if(obstaculo){auto_estado = 2;}
  
  #if EXIST_BLUETOOTH
  if (HC06.available()) {
    msg_blue = HC06.read();

   if(msg_blue == 'C'){ 
      #if EXIST_PID_OFFSET
      //desligando o PID do offset
      PID_OFFSET.SetMode(MANUAL);
      trava_pid_offset = false;
      #endif //EXIST_PID_OFFSET
      angulo_offset = 0; 

      motor_direito.para();
      motor_esquerdo.para();
      #if EXIST_PID_VEL 
      //desligando o PID da vlocidade 
      PID_VEL_D_PWM.SetMode(MANUAL);
      PID_VEL_E_PWM.SetMode(MANUAL);
      trava_pid_vel = false;
      #endif // EXIST_PID_VEL  
      pwm_e = 0;
      pwm_d = 0;

      trava_chao = true;   // permite que o mpu6050 calcule a tara novamente
      auto_estado = 0;   // faz com que o servo deixe as rodas retas
      switch_case = 0;   // volta para o menu inicial

    }else if(msg_blue == 'E'){
      // iniciar o andar do carro
      auto_estado = 1;

    }else if(msg_blue == 'D'){
      // freia fofo
      #if EXIST_PID_OFFSET
      //desligando o PID do offset
      PID_OFFSET.SetMode(MANUAL);
      trava_pid_offset = false;
      #endif //EXIST_PID_OFFSET
      // angulo_offset = 0;

      auto_estado = 2; // vai para o case que freia fofo

    }else if(msg_blue == 'F'){
      //alinha o servo
      angulo_offset = 0;
      auto_estado = 0;

    }else if(msg_blue == 'A'){
     trava_teste_reta = true;
    }
  }
  #endif //EXIST_BLUETOOTH


  switch (auto_estado){
    case 1:
      #if EXIST_PID_VEL 
      // parametros de PID para andar fofo
      PID_VEL_D_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      PID_VEL_E_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      #endif //#if EXIST_PID_VEL

      //define os valores de velocidade 
      vel_max_d = VEL_MAX;
      vel_max_e = VEL_MAX;
      vel_max = VEL_MAX;
      
      // regular a velocidade das rodas para curvas:

      if(angulo_visao > 6){
        // ** curva para esquerda **
        #if EXIST_PID_VEL
        // // parametros de PID para uma curva fofa
        // PID_VEL_D_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
        // PID_VEL_E_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
        #endif //#if EXIST_PID_VEL

        if(time_curva_e.atingiu_tempo()){
          vel_max_d = VEL_MAX;
          vel_max_e = VEL_MAX;
          trava_curva_esquerda = false;
        }
        if(trava_curva_esquerda){
          vel_max_d = VEL_MAX + 0.2;
          vel_max_e = VEL_MAX - 0.2;
        }

      }else if(angulo_visao < -6){
        // ** curva para direita **
        #if EXIST_PID_VEL
        // // parametros de PID para uma curva fofa
        // PID_VEL_D_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
        // PID_VEL_E_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
        #endif //#if EXIST_PID_VEL

        if(time_curva_d.atingiu_tempo()){
          vel_max_d = VEL_MAX;
          vel_max_e = VEL_MAX;
          trava_curva_direita = false;
        }
        if(trava_curva_direita){
          vel_max_d = VEL_MAX - 0.2;
          vel_max_e = VEL_MAX + 0.2; 
        }
      }else{
        trava_curva_esquerda = true;
        trava_curva_direita = true;
      }

      Andar();
      visao_controle();
    break;

    //------------------------------------------------------
    case 2:
      // **freiar fofo com PID**
      #if EXIST_PID_VEL 
      // // parametros de PID para uma curva fofa
      PID_VEL_D_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      PID_VEL_E_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      #endif //#if EXIST_PID_VEL 

      Frenagem_fofo();
    break;

    //------------------------------------------------------
    case 3: 
    //espaço reservado para baliza do carro
    break;
  
    //------------------------------------------------------
    default:
      servo.colocar_angulo(ANGULO_INICIAL);    
    break;
  }
  
}
#endif // EXIST_VISAO
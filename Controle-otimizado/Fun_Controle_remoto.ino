 // controle remoto do veiculo
void Controle_remoto(){
  
  #if EXIST_MPU6050
  if(trava_chao){Call_ref_chao();trava_chao = false;}
  #endif

  if(trava_ultrasson){if(obstaculo){remoto_estado = 2;}else{remoto_estado = 1;}}

  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'E'){
      // andar 
      farol_traseiro.apagar(); //acender os farois de freio
      trava_ultrasson = true;
      remoto_estado = 1;

    }else if(msg_blue == 'D'){
      // frenagem fofo
      seta_direita.acender();
      seta_esquerda.acender();
      trava_ultrasson = false;
      remoto_estado = 2;

    }else if(msg_blue == 'F'){
      //testar mudança de velocidade
       remoto_estado = 3;

    }else if(msg_blue == 'C'){
      // volta para o menu
      seta_direita.apagar();
      seta_esquerda.apagar();
      farol_traseiro.acender();
      farol_frente.apagar(); //acender os farois da frente
      
      motor_direito.para();
      motor_esquerdo.para();
      trava_ultrasson = false;
      pwm_e = 0;
      pwm_d = 0;
      switch_case = 0;
      remoto_estado = 0;
      auto_estado = 0;
      trava_pid_vel = false;
     
    }else if(msg_blue == '1'){
      angulo_servo = angulo_servo + 20;
      if(angulo_servo > angulo_maximo){angulo_servo = angulo_maximo;}
      

    }else if(msg_blue == '2'){
      angulo_servo = angulo_servo - 20;
      if(angulo_servo < angulo_minimo){angulo_servo = angulo_minimo;}
      
    }
  }

  if(obstaculo){remoto_estado = 2;}

  switch (remoto_estado){
    case 1:
      farol_traseiro.apagar();
      #if EXIST_PID_VEL 
      PID_VEL_D_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      PID_VEL_E_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      #endif //#if EXIST_PID_VEL 
      vel_max_d = VEL_MAX;
      vel_max_e = VEL_MAX;
      vel_max = VEL_MAX;

      // if(angulo_servo < ANGULO_ZERO -40){
      //   // ligar seta da direita
      //   seta_direita.piscar(500);
      // }else{seta_direita.acender();}  

      // if(angulo_servo > ANGULO_ZERO +40){
      //   //ligar seta da esquerda
      //   seta_esquerda.piscar(500);
      // }else{seta_esquerda.acender();}
      
      Andar();
    break;

    case 2:
      farol_traseiro.acender(); // acende o farol de freio
      #if EXIST_PID_VEL 
      PID_VEL_D_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      PID_VEL_E_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      #endif
      Frenagem_fofo();
    break;

    case 3:
    #if EXIST_PID_VEL 
      PID_VEL_D_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
      PID_VEL_E_PWM.SetTunings(KP_MCU, KI_MCU, KD_MCU);
    #endif 
      vel_max_d = VEL_MAX + 0.15;
      vel_max_e = VEL_MAX - 0.15; 
      Andar();
    break;

    case 4:
      
    break;
    
    default:
    break;
  }
  
  servo.colocar_angulo(angulo_servo);
  
}

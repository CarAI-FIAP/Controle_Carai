#if EXIST_VISAO
void Autonomo(){
  
  #if EXIST_MPU6050
  // faz a tara do mpu6050
  if(trava_chao){Call_ref_chao();trava_chao = false;}
  #endif //EXIST_MPU6050
 
  int volta_andar = 0;

  // freia fofo quando ultrassonico detecta um obstaculo
  if(trava_ultrasson){if(obstaculo){auto_estado = 2;}else{volta_andar++;}}

  if(trava_placa){if(placa_pare == 1){auto_estado = 2;}else{volta_andar++;}}

  if(trava_semafaro){if(semaforo == 1){auto_estado = 2;}else{volta_andar++;}}

  if(volta_andar == 3){auto_estado = 1;}


  if (HC06.available()) {
    // leitura do bluetooth
    msg_blue = HC06.read();

   if(msg_blue == 'C'){ 
      // **PARAR O CARRO E VOLTAR PARA O MENU**
      seta_direita.apagar();
      seta_esquerda.apagar();
      farol_traseiro.acender();
      farol_frente.apagar(); //acender os farois da frente
      // para os motores
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

      trava_ultrasson = false;  // impedir que o ultrassonico atue
      trava_placa = false;
      trava_semafaro = false;

      trava_chao = true;    // permite que o mpu6050 calcule a tara novamente
      auto_estado = 0;    // faz com que o servo deixe as rodas retas
      switch_case = 0;    // volta para o menu inicial

    }else if(msg_blue == 'E'){
      // **iniciar o andar do carro**
      farol_traseiro.apagar(); //acender os farois de freio
      trava_ultrasson = true; //ativa os sensores ultrassonicos
      trava_placa = true;
      trava_semafaro = true;
      auto_estado = 1; 

    }else if(msg_blue == 'D'){
      // **freiar fofo manualmente**
      seta_direita.acender();
      seta_esquerda.acender();
      
      trava_ultrasson = false; // impedir que o ultrassonico atue
      trava_placa = false;
      trava_semafaro = false;

      auto_estado = 2; // vai para o case que freia fofo

    }else if(msg_blue == 'F'){
      //alinha o servo
      auto_estado = 0;
    }
  }
  

  switch (auto_estado){
    case 1:
      #if EXIST_PID_VEL 
      // parametros de PID para andar fofo
      PID_VEL_D_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      PID_VEL_E_PWM.SetTunings(kp_mc, ki_mc, kd_mc);
      #endif //#if EXIST_PID_VEL

      //define os valores de velocidade durante uma reta 
      vel_max_d = VEL_MAX;
      vel_max_e = VEL_MAX;
      vel_max = VEL_MAX;

      // //-----------------------------------------------------------------------|
      //ajuste de velocidade das rodas durante as cuvas (METODO ANTIGO)        |
      if(angulo_visao > 40){ // melhor angulo = 40                             |
        // PID_VEL_D_PWM.SetTunings(kp_mcu, ki_mcu, kd_mcu);      //              |
        // PID_VEL_E_PWM.SetTunings(kp_mcu, ki_mcu, kd_mcu);      //              |
        //**curva para esquerda**                                              |
        vel_max_d = VEL_MAX +0.15;  // aumenta a velocidade da roda direita     |
        vel_max_e = VEL_MAX -0.15;  // diminui a velocidade da roda esquerda    |
        //                                                                     |
      }else if(angulo_visao < -40){ // melhor angulo = -40                     |
        // PID_VEL_D_PWM.SetTunings(kp_mcu, ki_mcu, kd_mcu);      //              |
        // PID_VEL_E_PWM.SetTunings(kp_mcu, ki_mcu, kd_mcu);      //              |                  
        //**curva para direita**                                               |
        vel_max_d = VEL_MAX -0.15; // diminui a velocidade da roda direita      |
        vel_max_e = VEL_MAX +0.15;  // aumenta a velocidade da roda esquerda     |
      }//                                                                      |
      //-----------------------------------------------------------------------|
      
      //-------------------------------------------------------------------------------------|
      //ajuste de velocidade das rodas durante as cuvas (METODO PROPORCIONAL)                |
      // int quant_aumento = ANGULO_CURVA_MAX - ANGULO_CURVA_MIN;   //                          |
      // int difere = angulo_visao - ANGULO_CURVA_MIN;   //                                     |
      // double uni_aumento = 0.2 / quant_aumento;     //                                       |
      // double fator_de_aumento = difere*uni_aumento; //                                       |
      // if(fator_de_aumento >= 0.2){fator_de_aumento = 0.2;} //                                |
      // //                                                                                     |
      // if(angulo_visao >= ANGULO_CURVA_MIN){ // melhor angulo = 40                            |
      //   //**curva para esquerda**                                                            |
      //   vel_max_d = VEL_MAX + fator_de_aumento;  // aumenta a velocidade da roda direita     |
      //   vel_max_e = VEL_MAX - fator_de_aumento;  // diminui a velocidade da roda esquerda    |
      //   //                                                                                   |
      // }else if(angulo_visao <= -ANGULO_CURVA_MIN){ // melhor angulo = -40                    |
      //   //**curva para direita**                                                             |
      //   vel_max_d = VEL_MAX - fator_de_aumento; // diminui a velocidade da roda direita      |
      //   vel_max_e = VEL_MAX + fator_de_aumento;  // aumenta a velocidade da roda esquerda    |
      // }//                                                                                    |                                                              
      //-------------------------------------------------------------------------------------|  

  
      if(angulo_visao <= -10){
        // ligar seta da direita
        seta_direita.piscar(500);
      }else{seta_direita.acender();}

      if(angulo_visao >= 10){
        //ligar seta da esquerda
        seta_esquerda.piscar(500);
      }else{seta_esquerda.acender();}
      
      farol_traseiro.apagar(); // apagar o farol de freio
      Andar();
      visao_controle();
    break;

    //------------------------------------------------------
    case 2:
      // **freiar fofo com PID**
      farol_traseiro.acender(); // acender o farol de freio
      
      #if EXIST_PID_VEL 
      // // parametros de PID para uma frenagem fofa
      PID_VEL_D_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      PID_VEL_E_PWM.SetTunings(KP_MFF, KI_MFF, KD_MFF);
      #endif //#if EXIST_PID_VEL 
      
      Frenagem_fofo();
    break;

    //------------------------------------------------------
    case 3: 
    //espaço reservado para baliza do carro.
    break;
  
    //------------------------------------------------------
    default:
      seta_direita.acender();
      seta_esquerda.acender();
      servo.colocar_angulo(ANGULO_INICIAL);    
    break;
  }
  
}
#endif // EXIST_VISAO
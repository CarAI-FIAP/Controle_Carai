#if EXIST_VISAO
void Autonomo(){

  #if EXIST_MPU6050
  if(trava_chao){Call_ref_chao();trava_chao = false;}
  #endif
  
  #if EXIST_BLUETOOTH
  if (HC06.available()) {
    msg_blue = HC06.read();
   if(msg_blue == 'C'){ 
      motor_direito.para();
      motor_esquerdo.para();
      trava_pid_vel = false;
      trava_pid_offset = false;
      pwm_e = 0;
      pwm_d = 0;
      trava_chao = true;
      remoto_estado = 0;
      auto_estado = 0;
      switch_case = 0;
    }else if(msg_blue == 'E'){
      auto_estado = 1;
    }
  }
  #endif
  
  if(obstaculo){auto_estado = 2;}

  switch (auto_estado){
    case 1:
      Andar();
      visao_controle();
    break;

    case 2:
      // freiar com PID
      Frenagem_fofo();
    break;

    case 3:

    break;
  
    default:
    break;
  }
  
}
#endif // EXIST_VISAO
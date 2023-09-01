 // controle remoto do veiculo
void Controle_remoto(){
  
  #if EXIST_MPU6050
  if(trava_chao){Call_ref_chao();trava_chao = false;}
  #endif

  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'E'){
      remoto_estado = 3;

    }else if(msg_blue == 'D'){
      // frenagem fofo
      remoto_estado = 2;

    }else if(msg_blue == 'F'){
      //sem ideia

    }else if(msg_blue == 'C'){
      motor_direito.para();
      motor_esquerdo.para();
      switch_case = 0;
      remoto_estado = 0;

    }else if(msg_blue == '1'){
      angulo_servo = angulo_servo + 5;
      if(angulo_servo > angulo_maximo){angulo_servo = angulo_maximo;}
      msg_blue = 0;

    }else if(msg_blue == '2'){
      angulo_servo = angulo_servo - 5;
      if(angulo_servo < angulo_minimo){angulo_servo = angulo_minimo;}
      msg_blue = 0;
    }
  }

  if(obstaculo){remoto_estado = 2;}

  switch (remoto_estado){
    case 1:
      Andar();
    break;

    case 2:
      Frenagem_fofo();
      if(obstaculo){}else{remoto_estado = 3;}
    break;

    case 3:
      Aceleracao_fofa();
    break;
  
    default:
    remoto_estado = 3;
    break;
    }
  servo.colocar_angulo(angulo_servo);
}

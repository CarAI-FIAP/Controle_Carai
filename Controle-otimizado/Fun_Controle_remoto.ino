 // controle remoto do veiculo
#if EXIST_CONTROLE_REMOTO
void Controle_remoto(){

  #if EXIST_MPU6050
  if(trava_chao){
    Call_ref_chao();
    trava_chao = false;
  }
  #endif
 
 if(obstaculo){
    motor_direito.para();
    motor_esquerdo.para();
 }else{
  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'E'){
      motor_direito.frente(pwm);
      motor_esquerdo.frente(pwm); 
    }else if(msg_blue == 'D'){
      motor_direito.para();
      motor_esquerdo.para();
    }else if(msg_blue == 'F'){
      motor_direito.tras(pwm);
      motor_esquerdo.tras(pwm);
    }else if(msg_blue == 'C'){
      motor_direito.para();
      motor_esquerdo.para();
      switch_case = 0;
    }else if(msg_blue == '1'){
      angulo_servo = angulo_servo + 5;
      if(angulo_servo > angulo_maximo){angulo_servo = angulo_maximo;}
      msg_blue = 0;
    }else if(msg_blue == '2'){
      angulo_servo = angulo_servo - 5;
      if(angulo_servo < angulo_minimo){angulo_servo = angulo_minimo;}
      msg_blue = 0;
    }else if(msg_blue == '3'){
      pwm++;
      if(pwm > 255){pwm = 255;}
      msg_blue = 0;      
    }else if(msg_blue == '4'){
      pwm--;     
      if(pwm < 0){pwm = 0;}        
      msg_blue = 0;         
    }
  }  
 }
  servo.colocar_angulo(angulo_servo); 
}
#endif // EXIST_CONTROLE_REMOTO
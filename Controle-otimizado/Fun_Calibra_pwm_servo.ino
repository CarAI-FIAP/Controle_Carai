// Ajuste do pwm
void Ajuste_pwm_manual(){
  #if EXIST_BLUETOOTH
  if (HC06.available()) {
    msg_blue = HC06.read();

    if(msg_blue == 'C'){
      // volta para o MENU
      motor_direito.para();
      motor_esquerdo.para();
      switch_case = 0;

    }else if(msg_blue == '3'){
      // aumenta vel
      pwm++;
      if(pwm > 255){pwm = 255;}
      msg_blue = 0;

    }else if(msg_blue == '4'){
      // diminui vel
      pwm--;     
      if(pwm < 0){pwm = 0;}        
      msg_blue = 0;         
    }       
    motor_direito.frente(pwm);
    motor_esquerdo.frente(pwm);   
  }
  #endif
}

/*******************************************************************/
// Ajusta o movimento do servo
void Ajuste_servo_manual(){
  #if EXIST_BLUETOOTH
  if (HC06.available()) {
    msg_blue = HC06.read();

    if(msg_blue == 'C'){
      // volta para o menu
      switch_case = 0;

    }else if(msg_blue == 'B'){
      // ceta angulo minimo
      angulo_minimo = angulo_servo;
      msg_blue = 0;

    }else if(msg_blue == 'A'){
      // ceta angulo maximo
      angulo_maximo = angulo_servo;
      msg_blue = 0;

    }else if(msg_blue == 'D'){
      // ceta angulo de tara
      angulo_zero = angulo_servo;
      msg_blue = 0;

    }else if(msg_blue == '1'){
      // aumenta angulo
      angulo_servo++;
      msg_blue = 0;

    }else if(msg_blue == '2'){
      // diminui angulo
      angulo_servo--;      
      msg_blue = 0;   

    }  
  }

  if(Serial.available() > 0) {angulo_servo = Serial.parseInt();}

  servo.colocar_angulo(angulo_servo);
  #endif 
}
  






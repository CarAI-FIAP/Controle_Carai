#if EXIST_CALIBRA_PWM_MANUAL
void Ajuste_pwm_manual(){
  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'B'){
      #if EXIST_MOTOR_DC 
      pwm_min = pwm;
      #endif // EXIST_MOTOR_DC 
      dados_print_HC06 += "Minimo ";
      dados_print_HC06 += "\t"; 
    }else if(msg_blue == 'A'){
      #if EXIST_MOTOR_DC 
      pwm_max = pwm;
      #endif // EXIST_MOTOR_DC 
      dados_print_HC06 += "Maximo ";
      dados_print_HC06 += "\t";      
    }else if(msg_blue == 'C'){
      dados_print_HC06 += "Calibração do motor finalizada ";
      dados_print_HC06 += "\t";
      #if EXIST_MOTOR_DC 
      motor_direito.para();
      motor_esquerdo.para();
      #endif // EXIST_MOTOR_DC 
      switch_case = 0;
    }else if(msg_blue == '3'){
      pwm++;
      if(pwm > 255){pwm = 255;}
      msg_blue = 0;      
    }else if(msg_blue == '4'){
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
    #endif  // EXIST_MOTOR_DC 
  }
}
#endif // EXIST_CALIBRA_PWM_MANUAL

/*******************************************************************/
// Ajusta o movimento do servo
#if EXIST_CALIBRA_SERVO
void Ajuste_servo_manual(){

  if (HC06.available()) {
    msg_blue = HC06.read();
    if(msg_blue == 'C'){
      dados_print_HC06 += "Calibração do servo finalizada ";
      dados_print_HC06 += " ";
      switch_case = 0;
    }else if(msg_blue == 'B'){
      angulo_minimo = angulo_servo;
      dados_print_HC06 += "Minimo ";
      dados_print_HC06 += " ";
    }else if(msg_blue == 'A'){
      angulo_maximo = angulo_servo;
      dados_print_HC06 += "Maximo ";
      dados_print_HC06 += " ";
    }else if(msg_blue == 'D'){
      angulo_zero = angulo_servo;
      dados_print_HC06 += "Zero ";
      dados_print_HC06 += " ";
    }else if(msg_blue == '1'){
      angulo_servo++;
      msg_blue = 0;      
    }else if(msg_blue == '2'){
      angulo_servo--;      
      msg_blue = 0;         
    } 
  dados_print_HC06 += "angulo do servo = ";
  dados_print_HC06 += String(angulo_servo);
  dados_print_HC06 += " ";
  }
  #if EXIST_SERVO
  servo.colocar_angulo(angulo_servo); 
  #endif // EXIST_SERVO
}
#endif // EXIST_CALIBRA_SERVO
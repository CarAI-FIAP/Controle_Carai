
void Andar(){
  #if EXIST_PID_VEL 
  //**andar com PID** 
  PID_VEL_D_PWM.SetMode(AUTOMATIC); // liga o PID direito
  PID_VEL_E_PWM.SetMode(AUTOMATIC); // liga o PID esquerdo
  trava_pid_vel = true; // aciona o PID
  
  motor_direito.frente(pwm_d);  //joga os valores de PWM do PID para o motor direito
  motor_esquerdo.frente(pwm_e); //joga os valores de PWM do PID para o motor esquerdo
  #endif //EXIST_PID_VEL 
    
  #if EXIST_NPID_VEL
    //**andar sem PID** 
    pwm_d = 150;  
    pwm_e = 150;
    motor_direito.frente(pwm_d);
    motor_esquerdo.frente(pwm_e);
}
  #endif //EXIST_NPID_VEL
}
//----------------------------------------------------------------------------------------------

void Frenagem_fofo(){
  #if EXIST_PID_VEL
  // ** FREIA FOFO COM PID **
  //coloca a velocidade a ser atngida como zero
  vel_max = 0; 
  vel_max_d = 0;
  vel_max_e = 0; 
 
 if(vel_md_f <= 0.09){ 
    // desliga o motor e o PID direito quando a velocidade é MUITO baixa
    pwm_d = 0;
    motor_direito.para();
    PID_VEL_D_PWM.SetMode(MANUAL);
    trava_pid_vel = false;
  }else{motor_direito.frente(pwm_d);}

  if(vel_me_f <= 0.09){
    // desliga o motor e o PID direito quando a velocidade é MUITO baixa
    pwm_e = 0;
    motor_esquerdo.para();
    PID_VEL_E_PWM.SetMode(MANUAL);
    trava_pid_vel = false;
  }else{motor_esquerdo.frente(pwm_e);} 
  
  #endif //EXIST_PID_VEL 

  //---***---***---***---***---***---**--**---***---***---**---***---***---***--
  //** FREIA FOFO SEM PID**
  #if EXIST_NPID_VEL
  if(vel_md_f != 0){
    if(time_frenagem_fofo_d.atingiu_tempo()){ 
      pwm_d--; 
      if(pwm_d < 0){pwm_d = 0;}
      motor_direito.frente(pwm_d);
    }
  }else{pwm_d = 0;motor_direito.para();}

  if(vel_me_f != 0){
    if(time_frenagem_fofo_e.atingiu_tempo()){
      pwm_e--;
      if(pwm_e < 0){pwm_e = 0;} 
      motor_esquerdo.frente(pwm_e);
    }
  }else{pwm_e = 0;motor_esquerdo.para();} 
  #endif //EXIST_NPID_VEL
}
//-----------------------------------------------------------------------------------------



void Andar(){
  #if EXIST_PID_VEL  
  PID_VEL_D_PWM.SetMode(AUTOMATIC);
  PID_VEL_E_PWM.SetMode(AUTOMATIC);
  trava_pid_vel = true;
  
  motor_direito.frente(pwm_d);
  motor_esquerdo.frente(pwm_e);
  #endif //EXIST_PID_VEL 
    
  #if EXIST_NPID_VEL
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
 
 int parado_e = 0;
 if(vel_md_f <= 0.09){ 
    // desliga o motor e o PID direito quando a velocidade é MUITO baixa
    pwm_d = 0;
    motor_direito.para();
    PID_VEL_D_PWM.SetMode(MANUAL);
    trava_pid_vel = false;
    parado_e = 1;
  }else{motor_direito.frente(pwm_d);}

  int parado_d = 0;
  if(vel_me_f <= 0.09){
    // desliga o motor e o PID direito quando a velocidade é MUITO baixa
    pwm_e = 0;
    motor_esquerdo.para();
    PID_VEL_E_PWM.SetMode(MANUAL);
    trava_pid_vel = false;
    parado_d = 1;
  }else{motor_esquerdo.frente(pwm_e);} 

  int paradoo = 0;
  paradoo = parado_d + parado_e;

  if(paradoo = 2){trava_ultrasson = true;}
   
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


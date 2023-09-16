
void Andar(){
  #if EXIST_PID_VEL 
  trava_pid_vel = true;
  
  motor_direito.frente(pwm_d);
  motor_esquerdo.frente(pwm_e);
  #endif //EXIST_PID_VEL 
    
  #if EXIST_NPID_VEL
  if(vel_md_f < vel_max ){
    pwm_d++; 
    if(pwm_d > 255){pwm_d = 255;}
    motor_direito.frente(pwm_d);
  }else if(vel_md_f > vel_max){
    pwm_d--;
    if(pwm_d < 0){pwm_d = 0;}
    motor_direito.frente(pwm_d);
  }

  if(vel_me_f < vel_max){
    pwm_e++; 
    if(pwm_e > 255){pwm_e = 255;}
    motor_esquerdo.frente(pwm_e);
  }else if(vel_me_f > vel_max){
    pwm_e--; 
    if(pwm_e < 0){pwm_e = 0;}
    motor_esquerdo.frente(pwm_e);
  }
  #endif //EXIST_NPID_VEL
}

void Frenagem_fofo(){
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
}

void Aceleracao_fofa(){
  int var_d = 0;
  if(vel_md_f < 0.1 ){  
    if(time_acelera_fofo_d.atingiu_tempo()){
      pwm_d++;
      if(pwm_d > 255){pwm_d = 255;}
      motor_direito.frente(pwm_d);
    }
  }else{var_d = 1;}

  int var_e = 0;
  if(vel_me_f < 0.1 ){
    if(time_acelera_fofo_e.atingiu_tempo()){
      pwm_e++;
      if(pwm_e > 255){pwm_e = 255;}
      motor_esquerdo.frente(pwm_e);
    }
  }else{var_e = 1;}
  if(var_e + var_d == 2){
    if(time_fofo.atingiu_tempo()){
      remoto_estado = 1; 
      auto_estado = 1;
    }
  }
}
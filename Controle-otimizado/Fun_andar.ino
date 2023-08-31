void Andar(){
  if(vel_md_f < VEL_MAX - 0.2){
    if(time_contrl_vel_d.atingiu_tempo()){
      pwm_d++; 
      if(pwm_d > 255){pwm_d = 255;}
      motor_direito.frente(pwm_d);
      motor_esquerdo.frente(pwm_d);
    }
  }else if(vel_md_f > VEL_MAX + 0.2){
    if(time_contrl_vel_d.atingiu_tempo()){
      pwm_d--; 
      if(pwm_d < 0){pwm_d = 0;}
      motor_direito.frente(pwm_d);
      motor_esquerdo.frente(pwm_d);
    }  
  }
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
  if(vel_md_f < VEL_MAX){  
    if(time_frenagem_fofo_d.atingiu_tempo()){
      pwm_d++;
      motor_direito.frente(pwm_d);
    }
  }else{var_d = 1;}

  int var_e = 0;
  if(vel_me_f < VEL_MAX){
    if(time_frenagem_fofo_e.atingiu_tempo()){
      pwm_e++;
      motor_esquerdo.frente(pwm_e);
    }
  }else{var_e = 1;}

  if(var_e + var_d == 2){remoto_estado = 1;}
}
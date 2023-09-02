
void Andar(){
  
  // if(trava_ponte){
  //   angulo_z_f_ant = angulo_z_f
  // }
  // angulo_z_f_ant = angulo_z_f

  if(vel_md_f < VEL_MAX - 0.15){
    if(time_contrl_vel_d.atingiu_tempo()){
      pwm_d++; 
      if(pwm_d > 255){pwm_d = 255;}
      motor_direito.frente(pwm_d);
    }
  }else if(vel_md_f > VEL_MAX + 0.15){
    if(time_contrl_vel_d.atingiu_tempo()){
      pwm_d--;
      #if EXIST_MPU6050
      if(angulo_z_f < -1){pwm_d = pwm_d*1.2;} 
      #endif 
      if(pwm_d < 0){pwm_d = 0;}
      motor_direito.frente(pwm_d);
    }  
  }

  if(vel_me_f < VEL_MAX - 0.15){
    if(time_contrl_vel_e.atingiu_tempo()){
      pwm_e++; 
      if(pwm_e > 255){pwm_e = 255;}
      motor_esquerdo.frente(pwm_e);
    }
  }else if(vel_me_f > VEL_MAX + 0.15){
    if(time_contrl_vel_e.atingiu_tempo()){
      pwm_e--; 
      if(pwm_e < 0){pwm_e = 0;}
      motor_esquerdo.frente(pwm_e);
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
    if(time_acelera_fofo_d.atingiu_tempo()){
      pwm_d++;
      if(pwm_d > 255){pwm_d = 255;}
      motor_direito.frente(pwm_d);
    }
  }else{var_d = 1;}

  int var_e = 0;
  if(vel_me_f < VEL_MAX){
    if(time_acelera_fofo_e.atingiu_tempo()){
      pwm_e++;
      if(pwm_e > 255){pwm_e = 255;}
      motor_esquerdo.frente(pwm_e);
    }
  }else{var_e = 1;}

  if(var_e + var_d == 2){remoto_estado = 1;}
}
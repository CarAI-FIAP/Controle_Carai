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
      motor_esquerdo.frente(pwm_d);
    }
  }else{
    pwm_d = 0;
    motor_direito.para();
    motor_esquerdo.para();
  }

  // if(vel_me_f != 0){
  //   if(time_frenagem_fofo_e.atingiu_tempo()){pwm_e--; motor_esquerdo.frente(pwm_e);}
  // }else{motor_esquerdo.para();} 
}

void Aceleracao_fofa(){
  if(vel_md_f < VEL_MAX){  
    if(time_frenagem_fofo_d.atingiu_tempo()){
      pwm_d++;
      motor_direito.frente(pwm_d);
      motor_esquerdo.frente(pwm_d);
    }
  }else{remoto_estado = 1;}

  // if(pwm_e < pwm_max){
  //   if(time_frenagem_fofo_e.atingiu_tempo()){pwm_e++;}
  // }else{remoto_estado = 1;}
}
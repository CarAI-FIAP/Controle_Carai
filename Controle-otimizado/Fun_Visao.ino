#if EXIST_VISAO
void  Visao_computacional(){
  if(Serial.available()){
    dados_visao = Serial.readStringUntil('\n');
    sscanf(dados_visao.c_str(), "%d,%d,%d,%d", &angulo_visao, &esquerda, &direita, &offset);
    angulo_visao_real = angulo_visao + angulo_zero;
    offset_double = offset;
    #if EXIST_VISAO_FILTRO
    offset_double = Filtro_offset.Media_movel(offset);
    angulo_visao_f = Filtro_visao.Media_movel(angulo_visao);
    angulo_visao_real = angulo_visao_f + angulo_zero;
    #endif //EXIST_VISAO_FILTRO
  }
}

void visao_controle(){
  
  #if EXIST_PID_OFFSET
  trava_pid_offset = true;
  #endif //EXIST_PID_OFFSET

  #if EXIST_NPID_OFFSET
  if(offset > 15){
    if(time_offset.atingiu_tempo()){
      angulo_offset--;
    }
  }else if(offset < -15){
    if(time_offset.atingiu_tempo()){
      angulo_offset++;
    }
  }else{angulo_offset = 0;}
  #endif

  angulo_visao_real = angulo_visao_real + angulo_offset;

  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  } 
  servo.colocar_angulo(angulo_visao_real); 


}
#endif //EXIST_VISAO
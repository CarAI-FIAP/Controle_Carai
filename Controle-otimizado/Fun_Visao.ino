#if EXIST_VISAO
void  Visao_computacional(){

  if(Serial.available()){
    delay(10);
    dados_visao = Serial.readStringUntil('\n');
    sscanf(dados_visao.c_str(), "%d,%d,%d,%d", &angulo_visao, &esquerda, &direita, &offset);
  
  
    //garantir que não tera picos de angulos fora do intervalo [-90,90]

    if(angulo_visao > 90){
      angulo_visao = angulo_visao_antigo;
    }else if(angulo_visao < -90){
      angulo_visao = angulo_visao_antigo;
    }
    angulo_visao_antigo = angulo_visao;

    //beckup para caso não tenha filtros 
    angulo_visao_f = angulo_visao*2;
    
    // angulo_visao_f = angulo_visao_f*(1+(angulo_visao_f/22));
    offset_double = offset;

    if(offset_antigo - offset_double > abs(20)){
      offset_double = offset_antigo;
    }

    offset_antigo = offset_double;

    #if EXIST_VISAO_FILTRO
    //filtro dos valores de offset e angulo recebido pela visão
    offset_double = Filtro_offset.Media_movel(offset);
    angulo_visao_f = Filtro_visao.Media_movel(angulo_visao);
    #endif //EXIST_VISAO_FILTRO

    // transforma o angulo recebido da visão para a versão real do servo 
    angulo_visao_real = angulo_visao_f + angulo_zero;
  }
  Serial.flush();
  
}

void visao_controle(){

  //PID do offset
  #if EXIST_PID_OFFSET
  trava_pid_offset = true;
  #endif //EXIST_PID_OFFSET

  //offset sem PID
  #if EXIST_NPID_OFFSET
  //garantir que apenas o offset de intervalo [-15,15] afete o angulo de offset
  if(offset_double > 3){
    if(offset_double < 40){
      if(time_offset.atingiu_tempo()){
      angulo_offset--;
      }
    }
  }else if(offset_double < -3){
    if(offset_double > -40){
      if(time_offset.atingiu_tempo()){
        angulo_offset++;
      }
    }
  }else{angulo_offset = 0;}

  // filtro para n deixar o offset sair do intervalo [-30,30]
  if(angulo_offset>60){angulo_offset = 60;}
  if(angulo_offset<-60){angulo_offset = -60;}
  
  // trava para garantir que em uma curva, caso a camera pare de mandar o offset
  if(offset_double >100){angulo_offset = -60;}
  if(offset_double <-100){angulo_offset = 60;}
  #endif

  // aplicação do angulo do offset no servo
  angulo_visao_real = angulo_visao_real + angulo_offset;

  // filtro de segurança para garantir que o angulo não ultrapasse os valores max e min
  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  } 

  angulo_servo = angulo_visao_real;

  servo.colocar_angulo(angulo_visao_real); 
  


}
#endif //EXIST_VISAO
#if EXIST_VISAO
void  Visao_computacional(){
  
  // comunicação com a visão computacional
  if(Serial.available()){
    delay(10);
    dados_visao = Serial.readStringUntil('\n');
    sscanf(dados_visao.c_str(), "%d",&angulo_offset);

    angulo_visao = angulo_offset;

    //manter apenas o offset na reta
    if(abs(angulo_visao) <= 3){angulo_visao = 0;}
  
    //garantir que não tera picos de angulos fora do intervalo [-90,90]
    if(abs(angulo_visao) > 90){angulo_visao = angulo_visao_antigo;} 
    
    // salva o angulo atual como antigo
    angulo_visao_antigo = angulo_visao;

    // detectando a exitencia de uma curva
    if(abs(angulo_visao) >= 20){detec_curva = true;}

    angulo_visao_f = angulo_visao;

    #if EXIST_VISAO_FILTRO
    //filtro dos valores do angulo recebido pela visão
    angulo_visao_f = Filtro_visao.Media_movel(angulo_visao);
    #endif //EXIST_VISAO_FILTRO

    // transforma o angulo recebido da visão para a versão real do servo 
    angulo_visao_real = angulo_visao_f + angulo_zero;

  }
  Serial.flush();
}

//-------------------------------------------------------------------------------------

void visao_controle(){

  // filtro de segurança para garantir que o angulo não ultrapasse os valores max e min
  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  } 
  
  angulo_servo = angulo_visao_real;

  if(time_servo.atingiu_tempo()){servo.colocar_angulo(angulo_visao_real);}
  // servo.colocar_angulo(angulo_visao_real);
  
}
#endif //EXIST_VISAO












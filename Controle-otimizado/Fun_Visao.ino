#if EXIST_VISAO
void  Visao_computacional(){
  
  // comunicação com a visão computacional
  if(Serial.available()){
    delay(10);
    dados_visao = Serial.readStringUntil('\n');

    char id = dados_visao.charAt(0);
    dados_visao = dados_visao.substring(1);

    if (id == 'A') {
      sscanf(dados_visao.c_str(), "%d,%d,%d,%d,%d,%d", &angulo_faixa, &angulo_offset, &offset, &esquerda, &direita, &valor_descartavel);

    } else if (id == 'B') {
      sscanf(dados_visao.c_str(), "%d,%d", &placa_pare, &semaforo);
    }
    
    // recolhe o dado de angulo vindo da visão
    angulo_visao = angulo_offset;

    //manter carro na reta
    if(abs(angulo_visao) <= 3){angulo_visao = 0;}
  
    //garantir que não tera picos de angulos fora do intervalo [-90,90]
    if(abs(angulo_visao) > 90){angulo_visao = angulo_visao_antigo;} 

    // evitar que o carro mude de direção do nada
    if(abs(angulo_visao_antigo - angulo_visao) > 30){
      if(angulo_visao_antigo*angulo_visao < 0){
        angulo_visao = angulo_visao_antigo;
      }
    }
    
    // salva o angulo atual como antigo
    angulo_visao_antigo = angulo_visao;

    // detectando a exitencia de uma curva
    if(abs(angulo_visao) >= 50){detec_curva = true;}
    
    // prepara uma variavel para caso da não existencia de um filtro
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

  // filtro de segurança para garantir que o angulo não ultrapasse os valores max e min fisicos
  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  } 
  
  // armazena o angulo que será printado
  angulo_servo = angulo_visao_real;

  if(time_servo.atingiu_tempo()){servo.colocar_angulo(angulo_visao_real);}  // atuação do servo com intervalo
  
  // servo.colocar_angulo(angulo_visao_real); // atuação do servo sem intervalo
  
}
#endif //EXIST_VISAO












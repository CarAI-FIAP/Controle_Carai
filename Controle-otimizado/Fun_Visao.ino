#if EXIST_VISAO
void  Visao_computacional(){
  
  // comunicação com a visão computacional
  if(Serial.available()){
    delay(10);
    dados_visao = Serial.readStringUntil('\n');
    sscanf(dados_visao.c_str(), "%d,%d,%d,%d", &angulo_visao, &esquerda, &direita, &offset);

    //utilizado para printar os valores recebidos pela visão
    angulo_teste = angulo_visao;
    offset_teste = offset;
    
    //manter apenas o offset na reta
    if(abs(angulo_visao) <= 2){angulo_visao = 0;}
  
    //garantir que não tera picos de angulos fora do intervalo [-90,90]
    if(abs(angulo_visao) > 90){angulo_visao = angulo_visao_antigo;} 
    
    // evitar mudança brusca de sentido
    if(abs(angulo_visao_antigo - angulo_visao) > 19){  // se a variação de angulo for maior que 19
      if(angulo_visao_antigo*angulo_visao < 0){
        angulo_visao = angulo_visao_antigo;
      }else{angulo_visao = angulo_visao_antigo;}
    }
    
    // salva o angulo atal como antigo
    angulo_visao_antigo = angulo_visao;

    // detectando a exitencia de uma curva
    if(abs(angulo_visao) >= 7){detec_curva = true;}

    //beckup para caso não tenha filtros
    // obs: o angulo real não é suficiente, logo estamos mutiplicando o angulo recebido por um fator
    angulo_visao_f = angulo_visao*FATOR_ANGULO_VISAO;

    #if EXIST_VISAO_FILTRO
    //filtro dos valores do angulo recebido pela visão
    angulo_visao_f = Filtro_visao.Media_movel(angulo_visao*FATOR_ANGULO_VISAO);
    #endif //EXIST_VISAO_FILTRO

    // transforma o angulo recebido da visão para a versão real do servo 
    angulo_visao_real = angulo_visao_f + angulo_zero;
    

    //--- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  ---
    
    // transformando o offset em double
    offset_double = offset;

    // evitar que o offset passe do limite 
    // if(abs(offset_double) > 150){offset_double = offset_antigo;}
    
    // para garantir que o offset não tenha uma variação muito grande indesejada
    // if(offset_antigo != 0){if(abs(offset_antigo - offset_double) > 20){offset_double = offset_antigo;}}
   
    // offset_antigo = offset_double;

    double offset_f = offset_double;

    #if EXIST_OFFSET_FILTRO
    offset_double = Filtro_offset.Media_movel(offset_f);
    #endif // EXIST_OFFSET_FILTRO

  }
  Serial.flush();
}

//-------------------------------------------------------------------------------------

void visao_controle(){

  //OFFSET COM PID
  #if EXIST_PID_OFFSET
  PID_OFFSET.SetControllerDirection(REVERSE);
  
  if(abs(angulo_visao) > 5){ 
    //desligando o offset
    PID_OFFSET.SetMode(MANUAL);
    trava_pid_offset = false;
    angulo_offset = 0;

  }else{  
    //ligando o offset
    PID_OFFSET.SetMode(AUTOMATIC);
    trava_pid_offset = true;
  }
  #endif //EXIST_PID_OFFSET

  //--- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  --- ***  ---
  // OFFSET SEM PID                                                                    |
  //                                                                                   |
  #if EXIST_NPID_OFFSET                                                             // |
  
  if (offset_double > 2){                                                            // |
    // tempo de 10ms para alinhar com o intervalo que a visão envia os dados        // |
    if(time_offset.atingiu_tempo()){angulo_offset++;}                               // |
                                                                                
  }else if(offset_double < -2){
    // tempo de 10ms para alinhar com o intervalo que a visão envia os dados
    if(time_offset.atingiu_tempo()){angulo_offset--;}

  }else{angulo_offset = 0;} 
 
  //zera o offset caso uma curva seja detectada
  if(abs(angulo_visao) > 5){angulo_offset = 0;}

  // filtro para n deixar o angulo offset sair do intervalo de angulo maximo e minimo
  if(angulo_offset > intervalo_maximo){angulo_offset = intervalo_maximo;}
  if(angulo_offset < intervalo_minimo){angulo_offset = intervalo_minimo;}

  #endif // EXIST_NPID_OFFSET


  //**************************************************************//
  
  // redundancia para desligar o offset no inicio da curva
  if(abs(angulo_visao) > 4){angulo_offset = 0;}

  // aplicação do angulo do offset no servo para retas 
  angulo_visao_real = angulo_visao_real + angulo_offset;

  // filtro de segurança para garantir que o angulo não ultrapasse os valores max e min
  if(angulo_visao_real < angulo_minimo){
    angulo_visao_real = angulo_minimo;
  }else if(angulo_visao_real > angulo_maximo){
    angulo_visao_real = angulo_maximo;
  } 

  // armazenando os dados para print
  angulo_servo = angulo_visao_real;
 
  //faz o carro ficar reto quando sair de uma curva
  // if(abs(angulo_visao) <= 5){
  //   if(detec_curva){
  //     angulo_visao_real = ANGULO_INICIAL;
  //     if(time_curva.atingiu_tempo()){detec_curva = false;}
  //   }
  // }

  if(trava_teste_reta){angulo_visao_real = ANGULO_INICIAL;}

  angulo_teste_2 = angulo_visao_real;
  if(time_servo.atingiu_tempo()){servo.colocar_angulo(angulo_visao_real);}
  // servo.colocar_angulo(angulo_visao_real);
  
}
#endif //EXIST_VISAO












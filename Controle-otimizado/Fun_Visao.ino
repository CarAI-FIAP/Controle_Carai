#if EXIST_VISAO
void  Visao_computacional(){
  if(Serial.available()){
    dados_visao = Serial.readStringUntil('\n');
    sscanf(dados_visao.c_str(), "%d,%d,%d,%d", &angulo_visao, &esquerda, &direita, &offset);
    angulo_visao_real = angulo_visao + angulo_zero;
    #if EXIST_VISAO_FILTRO
    angulo_visao_f = Filtro_visao.Media_movel(angulo_visao);
    angulo_visao_real = angulo_visao_f + angulo_zero;
    #endif //EXIST_VISAO_FILTRO
  }
}
#endif //EXIST_VISAO
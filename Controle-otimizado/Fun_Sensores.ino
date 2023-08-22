#if EXIST_Ultrassonico
void Distancia_Sensor(){
  distancia_1 = HCSR04_1.Calcula_dist();
  distancia_1f = distancia_1;
  #if EXIST_Ultrassonico_FILTRO
  distancia_1f = Filtro_HCSR04_1.Media_movel(distancia_1);
  #endif // EXIST_Ultrassonico_FILTRO
  obstaculo_1 = HCSR04_1.Detectar_obstaculo(distancia_1f);
  obstaculo = obstaculo_1;
  

  // distancia_2 = HCSR04_2.Calcula_dist();
  // distancia_2f = distancia_2;
  // #if EXIST_Ultrassonico_FILTRO
  // distancia_2f = Filtro_HCSR04_2.Media_movel(distancia_2);
  // #endif // EXIST_Ultrassonico_FILTRO
  // obstaculo_2 = HCSR04_2.Detectar_obstaculo(distancia_2f);

  // distancia_3 = HCSR04_3.Calcula_dist();
  // distancia_3f = distancia_3;
  // #if EXIST_Ultrassonico_FILTRO
  // distancia_3f = Filtro_HCSR04_3.Media_movel(distancia_3);
  // #endif // EXIST_Ultrassonico_FILTRO
  // obstaculo_3 = HCSR04_3.Detectar_obstaculo(distancia_3f);

  // if(obstaculo_1){
  //   obstaculo = true;
  // }else if(obstaculo_2){
  //   obstaculo = true;
  // }else if(obstaculo_3){
  //   obstaculo = true;
  // }else{obstaculo = false;}

}
#endif // EXIST_Ultrassonico


//*********************************************************************************
#if EXIST_ENCODER
void Encoder_call(){
  vel_md = encoder_D.velocidade();
  dist_total = encoder_D.dist_percorrida();
  #if EXIST_MEGA 
  vel_me = encoder_E.velocidade();
  #endif // EXIST_MEGA
}
#endif // EXIST_ENCODER
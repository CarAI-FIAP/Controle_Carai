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

//*********************************************************************************

void Call_ref_chao(){
  #if EXIST_GYROZ
  double zgx = 0;
  double x = 0;
  double zgz = 0;
  double z = 0;
  preInterval = millis();
  
  for(int i = 0; i < MEDIA_PARA_GIRO; i++){
    if(i % 1000 == 0){
      Serial.println("...");
    }
    while(i2cRead(0x3B, i2c_data, 14));
      zgx = (int16_t) ((i2c_data[8] << 8) | i2c_data[9]);
      x += ((float)zgx) / 131.0;
      gyroXoffset = x/MEDIA_PARA_GIRO;
      zgz = (int16_t) ((i2c_data[10] << 8) | i2c_data[11]);
      z += ((float)zgz) / 131.0;
      gyroZoffset = z/MEDIA_PARA_GIRO;
  }
  Serial.println("Concluido");
  trava_gyro = true;
  #endif
}

//************************************************************************************
#if EXIST_GYROZ
void Giroscopio(){
  while(i2cRead(0x3B, i2c_data, 14));
    gyroX = (int16_t) ((i2c_data[8] << 8) | i2c_data[9]);
    gyroX= gyroX/131.0;
    gyroZ = (int16_t) ((i2c_data[10] << 8) | i2c_data[11]);
    gyroZ= gyroZ/131.0;
  
  gyroX -= gyroXoffset;
  interval = (millis() - preInterval) * 0.001;
  angleX = (angleX + gyroX * interval);
  angulo_x = angleX;

  if(cont_offsetX < MEDIA_OFFSET){
   angulo_x_set = angulo_x_set + angulo_x;
   cont_offsetX = cont_offsetX + 1;
  }else{angulo_x_setoff = angulo_x_set/MEDIA_OFFSET;}

  gyroZ -= gyroZoffset;
  angleZ = (angleZ + gyroZ * interval);
  angulo_z = angleZ;

  if(cont_offsetZ < MEDIA_OFFSET){
   angulo_z_set = angulo_z_set + angulo_z;
   cont_offsetZ = cont_offsetZ + 1;
  }else{angulo_z_setoff = angulo_z_set/MEDIA_OFFSET;}
  
  preInterval = millis();
}
#endif
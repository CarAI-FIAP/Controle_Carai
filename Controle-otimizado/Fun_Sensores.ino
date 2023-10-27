#if EXIST_ULTRA
void Distancia_Sensor(){

  #if EXIST_ULTRA_MEIO
  distancia_1 = HCSR04_1.Calcula_dist(); //calcula distancia 
  distancia_1f = distancia_1;
  #if EXIST_ULTRA_FILTRO
  distancia_1f = Filtro_HCSR04_1.Media_movel(distancia_1);
  #endif // EXIST_ULTRA_FILTRO
  obstaculo_1 = HCSR04_1.Detectar_obstaculo(distancia_1f); //Detecta a existencia de um objeto
  if(obstaculo_1){
    detec_meio = 1;
  }else{detec_meio = 0;}
  obstaculo_1 = false;
  #endif //EXIST_ULTRA_MEIO
  
  #if EXIST_ULTRA_DIREITA
  distancia_2 = HCSR04_2.Calcula_dist();
  distancia_2f = distancia_2;
  #if EXIST_ULTRA_FILTRO
  distancia_2f = Filtro_HCSR04_2.Media_movel(distancia_2);
  #endif // EXIST_ULTRA_FILTRO
  obstaculo_2 = HCSR04_2.Detectar_obstaculo(distancia_2f);
  if(obstaculo_2){
    detec_direita = 1;
  }else{detec_direita = 0;} 
  obstaculo_2 = false;
  #endif //EXIST_ULTRA_DIREITA


  #if EXIST_ULTRA_ESQUERDA 
  distancia_3 = HCSR04_3.Calcula_dist();
  distancia_3f = distancia_3;
  #if EXIST_ULTRA_FILTRO
  distancia_3f = Filtro_HCSR04_3.Media_movel(distancia_3);
  #endif // EXIST_ULTRA_FILTRO
  obstaculo_3 = HCSR04_3.Detectar_obstaculo(distancia_3f);
  if(obstaculo_3){ 
    detec_esquerda = 1;
  }else{detec_esquerda = 0;} 
  obstaculo_3 = false;
  #endif // EXIST_ULTRA_ESQUERDA 

  
  if(detec_meio == 1){
    obstaculo = true;
  }else if(detec_direita == 1){
    obstaculo = true;
  }else if(detec_esquerda == 1){
    obstaculo = true;
  }else{obstaculo = false;}
  


}
#endif // EXIST_ULTRA

//*********************************************************************************
#if EXIST_ENCODER
void Encoder_call(){
  vel_md = encoder_D.velocidade();
  vel_md_f = vel_md;
  #if EXIST_ENCODER_FILTRO
  vel_md_f = Filtro_VEL_D.Media_movel(vel_md);
  #endif // EXIST_ENCODER_FILTRO

  double dist_direito = encoder_D.dist_percorrida();

  vel_me = encoder_E.velocidade();
  vel_me_f = vel_me;
  #if EXIST_ENCODER_FILTRO
  vel_me_f = Filtro_VEL_E.Media_movel(vel_me);
  #endif // EXIST_ENCODER_FILTRO

  double dist_esquerdo = encoder_E.dist_percorrida();

  dist_total = dist_direito + dist_esquerdo;
  dist_total = dist_total/2;
}
#endif // EXIST_ENCODER

//*********************************************************************************

#if EXIST_MPU6050
void Call_ref_chao(){
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

}

//-----------------------------------------------------------------------------------------

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
  angulo_x_f = angulo_x;
  #if EXIST_GYRO_FILTRO
  angulo_x_f = Filtro_giro_x.Media_movel(angulo_x);
  #endif // EXIST_GYRO_FILTRO

  if(cont_offsetX < MEDIA_OFFSET){
   angulo_x_set = angulo_x_set + angulo_x;
   cont_offsetX = cont_offsetX + 1;
  }else{angulo_x_setoff = angulo_x_set/MEDIA_OFFSET;}
  
  angulo_x_f = angulo_x_f - angulo_x_setoff;

  //-------------------------------------------

  gyroZ -= gyroZoffset;
  angleZ = (angleZ + gyroZ * interval);
  angulo_z = angleZ;
  angulo_z_f = angulo_z;
  #if EXIST_GYRO_FILTRO
  angulo_z_f = Filtro_giro_z.Media_movel(angulo_z);
  #endif // EXIST_GYRO_FILTRO

  if(cont_offsetZ < MEDIA_OFFSET_Z){
   angulo_z_set = angulo_z_set + angulo_z;
   cont_offsetZ = cont_offsetZ + 1;
  }else{angulo_z_setoff = angulo_z_set/MEDIA_OFFSET_Z;}

  angulo_z_f = angulo_z_f - angulo_z_setoff;
  
  preInterval = millis();
}
#endif //EXIST_MPU6050
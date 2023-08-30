//Print dos dados

void Prints(){
  #if EXIST_DADOS
  dados_print_PC += dado_menu;
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #if EXIST_MOTOR_DC 
  dados_print_PC += String(estado_motor);
  dados_print_PC += " ";
  dados_print_PC += String(pwm_d);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_MOTOR_DC

  #if EXIST_ENCODER
  dados_print_PC += String(vel_md_f);
  dados_print_PC += " ";
  dados_print_PC += String(vel_me_f);
  dados_print_PC += " ";
  dados_print_PC += String(dist_total);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_ENCODER

  #if EXIST_VISAO
  #if EXIST_VISAO_ORIGINAL
  dados_print_PC += String(angulo_visao);
  dados_print_PC += " ";
  #endif // EXIST_VISAO_ORIGINAL
  #if EXIST_VISAO_FILTRO
  dados_print_PC += String(angulo_visao_f);
  dados_print_PC += " ";
  #endif // EXIST_VISAO_FILTRO
  dados_print_PC += "| ";
  #endif // EXIST_VISAO

  #if EXIST_SERVO
  dados_print_PC += String(angulo_servo);
  dados_print_PC += " ";
  dados_print_PC += String(angulo_servo - angulo_zero);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_SERVO 

  #if EXIST_GYRO_PRINT
  dados_print_PC += String(angleX - angulo_x_setoff);
  dados_print_PC += " ";
  dados_print_PC += String(angleZ - angulo_z_setoff);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_GYRO_PRINT

  #if EXIST_INFRA
  dados_print_PC += String(dado_infra);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_INFRA
   
  #if EXIST_ULTRA
  dados_print_PC += String(detec);
  dados_print_PC += " ";
  dados_print_PC += String(distancia_2f);
  dados_print_PC += " ";
  dados_print_PC += String(distancia_1f);
  dados_print_PC += " ";
  dados_print_PC += String(distancia_3f);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_ULTRA

  #if EXIST_AJUSTE_GRAFICO
  dados_print_PC += String(250);
  dados_print_PC += "\t";
  dados_print_PC += String(0);
  dados_print_PC += "\t";
  dados_print_PC += String(-50);
  dados_print_PC += "\t";
  #endif // EXIST_AJUSTE_GRAFICO

  Serial.println(dados_print_PC);
  #endif // EXIST_DADOS
  
  dados_print_PC = " ";
}
//*************************************************************************************


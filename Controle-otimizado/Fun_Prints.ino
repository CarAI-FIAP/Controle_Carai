//Print dos dados
void Prints(){
  #if EXIST_DADOS
  dados_print_PC += dado_menu;
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #if EXIST_MOTOR_DC 
  dados_print_PC += String(estado_motor);
  dados_print_PC += " ";
  dados_print_PC += String(pwm);
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_MOTOR_DC

  #if EXIST_ENCODER
  dados_print_PC += String(vel_md);
  dados_print_PC += " ";
  #if EXIST_MEDIDA 
  dados_print_PC += "m/s";
  dados_print_PC += " ";
  #endif // EXIST_MEDIDA 
  #if EXIST_MEGA 
  dados_print_PC += String(vel_me);
  dados_print_PC += " ";
  #if EXIST_MEDIDA 
  dados_print_PC += "m/s";
  dados_print_PC += " ";
  #endif // EXIST_MEDIDA 
  #endif // EXIST_MEGA
  dados_print_PC += String(dist_total);
  dados_print_PC += " ";
  #if EXIST_MEDIDA 
  dados_print_PC += "m";
  dados_print_PC += " ";
  #endif // EXIST_MEDIDA 
  dados_print_PC += "| ";
  #endif // EXIST_ENCODER

  #if EXIST_VISAO
  #if EXIST_VISAO_ORIGINAL
  dados_print_PC += String(angulo_visao);
  #if EXIST_MEDIDA 
  dados_print_PC += "°";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  #endif // EXIST_VISAO_ORIGINAL
  #if EXIST_VISAO_FILTRO
  dados_print_PC += String(angulo_visao_f);
  #if EXIST_MEDIDA 
  dados_print_PC += "°";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  #endif // EXIST_VISAO_FILTRO
  #endif // EXIST_VISAO

  #if EXIST_SERVO
  dados_print_PC += String(angulo_servo);
  #if EXIST_MEDIDA 
  dados_print_PC += "°";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  dados_print_PC += String(angulo_servo - angulo_zero);
  #if EXIST_MEDIDA 
  dados_print_PC += "°";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_SERVO 
   
  #if EXIST_Ultrassonico
  dados_print_PC += String(detec);
  dados_print_PC += " ";
  #if EXIST_Ultrassonico_ORIGINAL
  dados_print_PC += String(distancia_1);
  #if EXIST_MEDIDA 
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA 
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_ORIGINAL 
  #if EXIST_Ultrassonico_FILTRO
  dados_print_PC += String(distancia_1f);
  #if EXIST_MEDIDA
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_FILTRO
  #if EXIST_MEGA
  #if EXIST_Ultrassonico_ORIGINAL
  dados_print_PC += String(distancia_2);
  #if EXIST_MEDIDA 
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA 
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_ORIGINAL 
  #if EXIST_Ultrassonico_FILTRO
  dados_print_PC += String(distancia_2f);
  #if EXIST_MEDIDA
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_FILTRO
  #if EXIST_Ultrassonico_ORIGINAL
  dados_print_PC += String(distancia_3);
  #if EXIST_MEDIDA 
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA 
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_ORIGINAL 
  #if EXIST_Ultrassonico_FILTRO
  dados_print_PC += String(distancia_3f);
  #if EXIST_MEDIDA
  dados_print_PC += "cm";
  #endif // EXIST_MEDIDA
  dados_print_PC += " ";
  #endif // EXIST_Ultrassonico_FILTRO
  #endif // EXIST_MEGA
  dados_print_PC += "| ";
  #endif // EXIST_Ultrassonico
  

  #if EXIST_AJUSTE_GRAFICO
  dados_print_PC += String(50);
  dados_print_PC += "\t";
  dados_print_PC += String(0);
  dados_print_PC += "\t";
  dados_print_PC += String(-50);
  dados_print_PC += "\t";
  #endif // EXIST_AJUSTE_GRAFICO
  Serial.println(dados_print_PC);
  #endif // EXIST_DADOS

  #if EXIST_BLUE
  if(dados_print_HC06 != " "){HC06.println(dados_print_HC06);}
  #endif // EXIST_BLUE
  
  dados_print_PC = " ";
  dados_print_HC06 = " ";
}
//*************************************************************************************


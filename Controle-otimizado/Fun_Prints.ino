//Print dos dados

void Prints(){
  #if EXIST_DADOS
  #if EXIST_SWITCH_DADOS
  dados_print_PC += String(switch_case); // case do menu de escolha
  // 1 = modo autonomo do veiculo
  // 2 = ajuste de pwm do carro
  // 3 = ajuste do servo 
  // 4 = contorle remoto
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif

  #if EXIST_MOTOR_DC_DADOS
  dados_print_PC += String(estado_motor); // estado geral dos motores. (ligado = 1 desligado = 0)
  dados_print_PC += " ";
  dados_print_PC += String(pwm_d);  // PWM do motor direito
  dados_print_PC += " ";
  dados_print_PC += String(pwm_e); // PWM do motor esquerdo
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_MOTOR_DC

  #if EXIST_ENCODER_DADOS
  dados_print_PC += String(vel_md_f);  // velocidade do motor direito em (m/s)
  dados_print_PC += " ";
  dados_print_PC += String(vel_me_f);  // velocidade do motor esquerdo em (m/s)
  dados_print_PC += " ";
  dados_print_PC += String(dist_total);  // distancia total percorrida pelo carro em (metros)
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_ENCODER_DADOS

  #if EXIST_VISAO_DADOS
  #if EXIST_OFFSET_DADOS
  dados_print_PC += String(offset_double); // Valor de offset dado pela visão [-200,200] negativo = defazagem para esquerda
  dados_print_PC += " ";  
  dados_print_PC += String(angulo_offset); // angulo acrescentado ao angulo que vai ao servo
  dados_print_PC += " ";
  #endif //EXIST_OFFSET_DADOS
  #if EXIST_VISAO_ANGULO_DADOS
  dados_print_PC += String(angulo_visao_f); // angulo recebido pela visão [-90,90]
  dados_print_PC += " ";  
  dados_print_PC += String(angulo_visao_real - angulo_offset); // angulo real que será utilizado pelo servo (sem a tara) [angulo min, angulo max] 
  dados_print_PC += " ";
  #endif // EXIST_VISAO_ANGULO_DADOS
  dados_print_PC += "| ";
  #endif // EXIST_VISAO_DADOS

  #if EXIST_SERVO_DADOS
  dados_print_PC += String(angulo_servo); // angulo real que o servo utilizará [angulo min, angulo max]
  dados_print_PC += " ";
  dados_print_PC += String(angulo_servo - angulo_zero); // angulo do servo com "tara" [-90,90]
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_SERVO_DADOS

  #if EXIST_GYRO_DADOS
  dados_print_PC += String(angulo_x_f); // angulo de rotação do carro (diferente do angulo do servo)
  dados_print_PC += " ";
  dados_print_PC += String(angulo_z_f);  // angulo de inclinação do carro 
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_GYRO_DADOS

  #if EXIST_INFRA_DADOS
  dados_print_PC += String(dado_infra_d); // estado de detecção da linha do infra direito (detectado = 1, não detectado = 0)
  dados_print_PC += " ";
  dados_print_PC += String(dado_infra_e); // estado de detecção da linha  do infra esquerdo (detectado = 1, não detectado = 0)
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_INFRA_DADOS
   
  #if EXIST_ULTRA_DADOS
  dados_print_PC += String(detec_meio); // informa a deteção do utrass. do meio (detectado = 1, não detectado = 0)
  dados_print_PC += " ";
  dados_print_PC += String(detec_direita); // informa a deteção do utrass. da direita (detectado = 1, não detectado = 0)
  dados_print_PC += " ";
  dados_print_PC += String(detec_esquerda); // informa a deteção do utrass. da esquerda (detectado = 1, não detectado = 0)
  dados_print_PC += " ";
  dados_print_PC += String(distancia_1f); // distancia medida pelo ultrass. do meio em (cm)
  dados_print_PC += " ";
  dados_print_PC += String(distancia_2f); // distancia medida pelo ultrass. da direita em (cm)
  dados_print_PC += " ";
  dados_print_PC += String(distancia_3f); // distancia medida pelo ultrass. daesquerda em (cm)
  dados_print_PC += " ";
  dados_print_PC += "| ";
  #endif // EXIST_ULTRA_DADOS

  dados_print_PC += String(esquerda); // distancia medida pelo ultrass. daesquerda em (cm)
  dados_print_PC += " ";
  dados_print_PC += String(direita); // distancia medida pelo ultrass. daesquerda em (cm)
  dados_print_PC += " ";
  dados_print_PC += "| ";

  #if EXIST_AJUSTE_GRAFICO
  dados_print_PC += String(1.2); 
  dados_print_PC += "\t";
  dados_print_PC += String(0);
  dados_print_PC += "\t";
  #endif // EXIST_AJUSTE_GRAFICO

  if(time_print.atingiu_tempo()){
  Serial.println(dados_print_PC);
  Serial2.println(dados_print_PC);
  }
  #endif // EXIST_DADOS
  
  dados_print_PC = " ";
}
//*************************************************************************************


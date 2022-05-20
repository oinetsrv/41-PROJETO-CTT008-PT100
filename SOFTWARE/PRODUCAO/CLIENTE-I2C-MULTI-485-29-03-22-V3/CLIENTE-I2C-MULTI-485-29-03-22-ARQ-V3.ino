/* ===========================================================================================
   Projeto: Termometria assistida por PT100
   
   Autor: Eng. Stenio Rodrigues         Data:  março de 2022.

================================================================================= */
// =================================================================================
// --- Bibliotecas Auxiliares ---

// =================================================================================
// --- Mapeamento de Hardware ---
 
// =================================================================================
// --- Instâncias ---
 
// =================================================================================
// --- Protótipo das Funções ---

// Monta média inicial.
void partiu             (int conversor                                  );
// Chama média móvel da leitura do conversor de acordo com quantidade de sensores
void media_leitura      (int conversor                                  );
// Realiza a média móvel de forma genérica
float moving_average    (float sig_G, int identidade                    );
// Debug serial com dados brutos do conversor
void imprimir_serial    (                                               );
// Abre a rede 485 e envia dados.
void Proto_485_V2       (String end_cliente, String msg_cliente         );
// Faz tratamento básico caso algum sensor não responda preenche vetor com zero.
void Proto_485_V4       (char *testar, int tamanho                      );
//Coloca marcador de endereço no vetor de dados 
void Proto_485_V3       (int ident_sensor_G                             );
// função que faz a conversão do sinal analógico digitalizado em temperatura Graus
float Conversor_mv_temp (float off_set_G, float f_calib_G, float results_G);
// define endereço do micro baseado nos pinos de entrada baseado em binario e com marcador de quantos sensores online
void Gerenciador_endereco(                                              );
// envia solicitação na rede 485 para cliente responder dados, analisa ultimo cliente que enviou
void Gerenciador_rede    (                                              );

// =================================================================================
// --- Desenvolvimento das Funções ---
// =================================================================================

void partiu                 (int conversor                                      ){
        int opa=5;
        for (int i = 0; i < 10; i++)
        {
            adc_off_set_G = 42; 
            media_leitura(conversor);
        }// end for
}// end partiu
// =================================================================================

void media_leitura          (int conversor                                      ){
  for (int i = 0; i < num + 1; ++i) {
        wdt_reset(); 
        switch (conversor) {
        case 1:
                ident_sensor_G = 1;
                results1_G = moving_average(ads_48.readADC_Differential_0_1(), ident_sensor_G);
                ident_sensor_G = 2;
                results2_G = moving_average(ads_48.readADC_Differential_2_3(), ident_sensor_G);
            break;
        case 8:
                ident_sensor_G = 1;
                results1_G = moving_average(ads_48.readADC_Differential_0_1(), ident_sensor_G);
                ident_sensor_G = 2;
                results2_G = moving_average(ads_48.readADC_Differential_2_3(), ident_sensor_G);
                ident_sensor_G = 3;
                results3_G = moving_average(ads_49.readADC_Differential_0_1(), ident_sensor_G);
                ident_sensor_G = 4;
                results4_G = moving_average(ads_49.readADC_Differential_2_3(), ident_sensor_G);
                ident_sensor_G = 5;
                results5_G = moving_average(ads_4A.readADC_Differential_0_1(), ident_sensor_G);
                ident_sensor_G = 6;
                results6_G = moving_average(ads_4A.readADC_Differential_2_3(), ident_sensor_G);
                ident_sensor_G = 7;
                results7_G = moving_average(ads_4B.readADC_Differential_0_1(), ident_sensor_G);
                ident_sensor_G = 8;
                results8_G = moving_average(ads_4B.readADC_Differential_2_3(), ident_sensor_G);  
                ident_sensor_G = 0;
            break;
        }// end switch case
        ident_sensor_G = 0;
    } // end for 
}//end media_leitura()
// =================================================================================

float moving_average        (float sig_G, int identidade                        ){
    float acc = 0.00; //acumulador
    int ident = identidade;
    float sig = sig_G;
      switch (ident) {
        case 1:
            values1_G[0] = (float) sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values1_G[i] = values1_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values1_G[i];   }
            break;
        case 2:
            values2_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values2_G[i] = values2_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values2_G[i];   }
            break;
        case 3:
            values3_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values3_G[i] = values3_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values3_G[i];   }
            break;
        case 4:
            values4_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values4_G[i] = values4_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values4_G[i];   }
            break;
        case 5:
            values5_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values5_G[i] = values5_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values5_G[i];   }
            break;
        case 6:
            values6_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values6_G[i] = values6_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values6_G[i];   }
            break;
        case 7:
            values7_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values7_G[i] = values7_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values7_G[i];   }
            break;
        case 8:
            values8_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values8_G[i] = values8_G[i - 1]; }
            for (int i = 0; i < num; i++) { acc += (float) values8_G[i];   }
            break;
        default:
            values_G[0] = sig; //carrega o sinal no primeiro elemento do vetor
            for (int i = num; i > 0; i--) { values_G[i] = values_G[i - 1];   }
            for (int i = 0; i < num; i++) { acc += (float) values_G[i];    }
            return acc / num;
            break;
      }// end switch (ident)
    return (float) acc / (float) num;
} //end moving_average()
// =================================================================================

float Conversor_mv_temp     (float off_set_G, float f_calib_G, float results_G  ){
  //passar off_set f_calib adc_lido retornar result
    float off_set  = off_set_G;
    float f_calib  = f_calib_G;
    float result   = results_G;
    float adc_calc =       0.000;
        adc_calc = (float) result - (float)off_set;
        adc_calc = 1000 * ( (float) adc_calc / (float) f_calib );
    return adc_calc;
} // end temperatura_bruto
// =================================================================================

void Proto_485_V3           (int ident_sensor_G                                 ){
    int P_sensor_G  = ident_sensor_G;
    switch (ident_sensor_G) {
            case 1:
                str_sensor = "A";
                break;
            case 2:
                str_sensor = "B"; 
                break;
            case 3:
                str_sensor = "C"; 
                break;
            case 4:
                str_sensor = "D"; 
                break;
            case 5:
                str_sensor = "E"; 
                break;
            case 6:
                str_sensor = "F"; 
                break;
            case 7:
                str_sensor = "G"; 
                break;
            case 8:
                str_sensor = "H"; 
                break;
    } // end SWITCH
} // end Proto_485_V3
// =================================================================================

void Proto_485_V4           (char *testar, int tamanho                          ){
        int tam = tamanho;
        char manobra [tam];
        //Serial.println(" BRUTA! ");
        for (size_t i = 0; i <= 41; i++) manobra [i] = testar [i];
        //         1   2   3   4   5   6   7   8
        //String  "A000B000C000D000E000F000G000H000";
            if (str1[0 ] != 'A' ){
                manobra[0 ] = 'A';
                manobra[1 ] = '0';
                manobra[2 ] = '.';
                manobra[3 ] = '0';
                manobra[4 ] = '0';
            } 
            if (str1[5 ] != 'B' ){
                manobra[5 ] = 'B';
                manobra[6 ] = '0';
                manobra[7 ] = '.';
                manobra[8 ] = '0';
                manobra[9 ] = '0';
            }  
            if (str1[10] != 'C' ){
                manobra[10] = 'C';
                manobra[11] = '0';
                manobra[12] = '.';
                manobra[13] = '0';
                manobra[14] = '0';
            }
            if (str1[15] != 'D' ){
                manobra[15] = 'D';
                manobra[16] = '0';
                manobra[17] = '.';
                manobra[18] = '0';
                manobra[19] = '0';
            }
            if (str1[20] != 'E' ){
                manobra[20] = 'E';
                manobra[21] = '0';
                manobra[22] = '.';
                manobra[23] = '0';
                manobra[24] = '0';
            }
            if (str1[25] != 'F' ){
                manobra[25] = 'F';
                manobra[26] = '0';
                manobra[27] = '.';
                manobra[28] = '0';
                manobra[29] = '0';
            }
            if (str1[30] != 'G' ){
                manobra[30] = 'G';
                manobra[31] = '0';
                manobra[32] = '.';
                manobra[33] = '0';
                manobra[34] = '0';
            }
            if (str1[35] != 'H' ){
                manobra[35] = 'H';
                manobra[36] = '0';
                manobra[37] = '.';
                manobra[38] = '0';
                manobra[39] = '0';
            }
        for (size_t i = 0; manobra[i]!='\0'; i++) str1 [i] = manobra [i];
}// end Proto_485_V4
// =================================================================================

void Proto_485_V2           (String end_cliente, String msg_cliente             ){
    digitalWrite(MASTER, HIGH);
    delay(1);
    mySerial.print(msg_cliente);
    mySerial.flush();
    mySerial.print(end_cliente);
    mySerial.flush();
    delay(10);
    digitalWrite(MASTER, LOW);
    // DEBUG
    Serial.print(msg_cliente);
    Serial.flush();
} // end Proto_485_V2
// =================================================================================

void imprimir_serial        () {
  float    teste    =0;  
    Serial.print  ("ads_48 S1: ");
    teste = ads_48.readADC_Differential_0_1();
    Serial.println(teste);
    Serial.print  ("ads_48 S2: ");
    teste = ads_48.readADC_Differential_2_3();
    Serial.println(teste);
    if(conversor==8){
        Serial.print  ("ads_49 S3: ");
        teste = ads_49.readADC_Differential_0_1(); 
        Serial.println(teste);
        Serial.print  ("ads_49 S4: ");
        teste = ads_49.readADC_Differential_2_3();
        Serial.println(teste);
        Serial.print  ("ads_4A S5: ");
        teste = ads_4A.readADC_Differential_0_1(); 
        Serial.println(teste);
        Serial.print  ("ads_4A S6: ");
        teste = ads_4A.readADC_Differential_2_3();
        Serial.println(teste);
        Serial.print  ("ads_4B S7: ");
        teste = ads_4B.readADC_Differential_0_1(); 
        Serial.println(teste);
        Serial.print  ("ads_4B S8: ");
        teste = ads_4B.readADC_Differential_2_3();
        Serial.println(teste);
    }// end if
    Serial.print  ("\n");
    Serial.write  (0xC);
    Serial.flush (); 
} // end imprimir_serial
// =================================================================================

void imprimir_serial_temp   (){
    String temp_msg_padrao = "";
    float t_m = 0.0;
    String temp_ = "";
    // rotina de leitura conversor A/D
    ident_sensor_G = 1;
    results1_G = moving_average(ads_48.readADC_Differential_0_1(), ident_sensor_G);
    t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results1_G            );
    // dividir por mil para colocar casas decimais
    t_m = float(t_m / 1000);
    if (t_m<0) t_m = 0;
    ////if (t_m>98) t_m = 98;
    if (t_m <10)temp_msg_padrao = String(t_m, 2);
    if (t_m>=10)temp_msg_padrao = String(t_m, 1);
   // temp_msg_padrao = String(t_m, 1); // converte e arredonda// converte todos as casas String(t_m, DEC);
    Proto_485_V3(ident_sensor_G); // COLOCA ENDEREÇO SENSOR
    temp_.concat(str_sensor); 
    temp_.concat(temp_msg_padrao);
    ident_sensor_G =  2;
    results2_G = moving_average(ads_48.readADC_Differential_2_3(), ident_sensor_G);
    t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results2_G            );
    t_m = float(t_m / 1000);
    if (t_m<0) t_m = 0;
    //if (t_m>98) t_m = 98;
    if (t_m <10)temp_msg_padrao = String(t_m, 2);
    if (t_m>=10)temp_msg_padrao = String(t_m, 1);
    Proto_485_V3(ident_sensor_G);  
    temp_.concat(str_sensor); 
    temp_.concat(temp_msg_padrao);
    if(conversor == 8){
        //Serial.print(" Rodou ate aqui | \n");
        ident_sensor_G =  3;
        results3_G = moving_average(ads_49.readADC_Differential_0_1(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results3_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
        ident_sensor_G =  4;
        results4_G = moving_average(ads_49.readADC_Differential_2_3(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results4_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
        ident_sensor_G =  5;
        results5_G = moving_average(ads_4A.readADC_Differential_0_1(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results5_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
        ident_sensor_G =  6;
        results6_G = moving_average(ads_4A.readADC_Differential_2_3(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results6_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
        ident_sensor_G =  7;
        results7_G = moving_average(ads_4B.readADC_Differential_0_1(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results7_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
        ident_sensor_G =  8;
        results8_G = moving_average(ads_4B.readADC_Differential_2_3(), ident_sensor_G);
        t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results8_G            );
        t_m = float(t_m / 1000);
        if (t_m<0) t_m = 0;
        //if (t_m>98) t_m = 98;
        if (t_m <10)temp_msg_padrao = String(t_m, 2);
        if (t_m>=10)temp_msg_padrao = String(t_m, 1);
        Proto_485_V3(ident_sensor_G);  
        temp_.concat(str_sensor); 
        temp_.concat(temp_msg_padrao);
    }// end if
    wdt_reset();
    // criar testador de string para validar todas as posiçoes
    temp_.toCharArray(str1, temp_.length()+1);
    Serial.print("\n");
    Serial.print(str1);
    Serial.println(" ");
    for (size_t i = 0;  str1[i]!='\0'; i++) str1[i] = "";
    // fim debug
}// end  imprimir_serial_temp
// =================================================================================

void Gerenciador_endereco(){
       if(bit_A == 0 && bit_B == 0){
                conversor = 8; // tem todos sensores
        }else{
                conversor = 1; // nao tem todos sensores
        }// end else
            // clinte com todos sensores definir endereço
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 1;
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 2;
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 3;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 4;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 5;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 6;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 7;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 8;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 9;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 10;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 11;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 12;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 13;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 14;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 15;
   }// end Gerenciador_endereco
// =================================================================================

void Gerenciador_rede(){
    wdt_reset();
    String end_cliente = "P00X*";
                    //   1   2   3   4   5   6   7   8
    String msg_padrao = "A000B000C000D000E000F000G000H000";
    msg_padrao = str1;
    int tam_msg_485 = 41;
    char montar_msg[tam_msg_485];
    // conversão bruta //
    int posicao_inicial = 0,
        posicao_final = 0;
    int conta_caracter = 0;
    digitalWrite(MASTER, LOW);
// ****************************
    // para rede 485
       // while (mySerial.available()) {
        //   char request = mySerial.read();
        //while (Serial.available()) {
        //   char request = Serial.read();

        // para debug no serial 232
         while (mySerial.available()) {
            char request = mySerial.read();
//*******************************************
            str.concat(request);
            int tamanho_string = str.length();
            charRecebida[conta_caracter] = request;
            if (charRecebida[conta_caracter]   == '\0' && conta_caracter > 5) {
                //Serial.println  ("AOBA: "         );
                int     final_msg   =  0, 
                        P_msg       =  0;
                for (size_t i = 0;  charRecebida[i]!='\0'; i++){
                     if (charRecebida[i]   == 'P' && charRecebida[i+1]   == '0')
                     {
                        P_msg = i;
                        //Serial.print    ("MARCADOR!: " );
                        //Serial.println  (P_msg);
                     }// END IF
                     if (charRecebida[i]   == '*')
                     {
                        final_msg = i+1;
                        //Serial.print    ("final_msg: " );
                        //Serial.println  (final_msg);
                     }// END IF
                }// END FOR
                 
                       for (size_t i = P_msg; i < P_msg+5; i++)
                       {
                           Serial.print  (charRecebida[i]);
                       }// END IF
                       
                conta_caracter = 0;
                str.remove(0, str.length());
                for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";

                break;
            }// END IF
            conta_caracter++;
        } // end while
            if (conta_caracter >45)
            {
                Serial.print  ("\nAPAGOU: \n" );
                str.remove(0, str.length());
                for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";
                str = "";
                ////limpa string e char mesmo sendo temporaria////////
                conta_caracter = 0;
            }// end if   
}//void Gerenciador_rede()

// =================================================================================
// =================================================================================
// =================================================================================
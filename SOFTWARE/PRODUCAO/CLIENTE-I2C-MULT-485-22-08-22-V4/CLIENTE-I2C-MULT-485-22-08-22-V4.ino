/* ===========================================================================================
    Projeto: Termometria assistida por sensores PT100
    Autor: Eng. Stenio Rodrigues         Data:  MARÇO de 2022.
    Versão: PRODUÇÃO

   Descrição do projeto: Sensor PT100 instalado dentro de uma sonda
   composta por cabo de dados + sensores + alma de aço revestida por mangueira politileno.
   Conversão do sinal analogico digital utilizando ADS1115-4 composta por até 8 sensores
   Dados transferidos para microcontrolador ATMEGA328P U  
   O microcontrolador gerencia a rede I2C e converte para unidade decimal
   Envia dados via RS485 multipontos via requisição
    
   Exemplo de vetor resposta:
    "A000B000C000D000E000F000G000H000P001*"
    PRIMEIRO CARACTER:          Corresponde ao endereço do sensor posicionado na sonda  
    SEGUNDO AO QUARTO CARACTER: Corresponde a leitura do sensor posionado na sonda   
    O vetor repete o patrão até o ultimo sensor listado nesse controlador
    PENULTIMO CARACTER:         Corresponde ao endereço do controlador na rede
    ULTIMO CARACTER:            Corresponde ao marcador final do vetor, aponta para o final da msg.
    CARACTERES "ABCDEFGH":      Corresponde a marcadores de endereço dos sensores
    CARACTER "P":               Corresponde a marcador de endereço do controlador           
================================================================================= */
/*
notas: Desenvolvimento utilizando o VSCODE         
        copilar o codigo terminal definir se UNO OU MEGA!!!
        arduino-cli compile -b arduino:avr:mega -v 
        identificar em qual porta está o arduino
        arduino-cli board list
        enviar depois de copilado
        arduino-cli upload -b arduino:avr:mega  -p /dev/ttyUSB0 -v
        para mais informações
        arduino-cli --help
        USANDO VSCODE
            F1 + arduino select sketch (pasta) marque o arquivo que vc quer copilar
            F1 + arduino: verify, upload, open, close serial monitor, board manager, change baud rate,
            change board type, select programmer, select serial port,
        PARA IDENTAR O CODIGO ALT + SHIFT + F no vscode
// =================================================================================*/
// --- Bibliotecas Auxiliares ---
    #include <SoftwareSerial.h>     // biblioteca que gerencia multiplas portas
    #include <Adafruit_ADS1X15.h> // CONVERSOR AD
    #include <EEPROM.h>     // EEPROM
    #include <avr/io.h>  // wachdog
    #include <avr/wdt.h> // wachdog
    #include <math.h> // tirar raiz do RMS 
// =================================================================================
// --- Mapeamento de Hardware ---
    #define rxPin 10    // define pino D10 como Rx
    #define txPin 11    // define pino D11 como Tx
    #define MASTER 13   // SE HIGH FALA SE LOW ESCUTA!!! 13

    #define Pbit_0 2    //  
    #define Pbit_1 3    // 
    #define Pbit_2 4    //  
    #define Pbit_3 5    //  
    #define Pbit_A 6    //  
    #define Pbit_B 7    //  
    /* ENDEREÇO PARA 8 SENSORES
       bit_B bit_A bit_3 bit_2 bit_1 bit_0    ENDEREÇO RESULTANTE
       
         0     0    0     0     0     1    // ENDEREÇO 01
         0     0    0     0     1     0    // ENDEREÇO 02
         0     0    0     0     1     1    // ENDEREÇO 03
         0     0    0     1     0     0    // ENDEREÇO 04
         0     0    0     1     0     1    // ENDEREÇO 05
         0     0    0     1     1     0    // ENDEREÇO 06
         0     0    0     1     1     1    // ENDEREÇO 07
         0     0    1     0     0     0    // ENDEREÇO 08
         0     0    1     0     0     1    // ENDEREÇO 09
         0     0    1     0     1     0    // ENDEREÇO 10
         0     0    1     0     1     1    // ENDEREÇO 11
         0     0    1     1     0     0    // ENDEREÇO 12
         0     0    1     1     0     1    // ENDEREÇO 13
         0     0    1     1     1     0    // ENDEREÇO 14
         0     0    1     1     1     1    // ENDEREÇO 15

    */

// =================================================================================
// =================================================================================
// --- Instâncias ---
    SoftwareSerial mySerial(rxPin, txPin);  // instanciar objeto de SoftwareSerial 
    Adafruit_ADS1015 ads_48;                // biblioteca externa do conversor A/D
    Adafruit_ADS1015 ads_49;
    Adafruit_ADS1015 ads_4A;
    Adafruit_ADS1015 ads_4B;

// =================================================================================
// =================================================================================
// --- Variáveis Globais ---
#define num 10 //número de iterações da média móvel
int conversor           =    0,                             // quantidade de sensores na rede
    silencio            =    0,                             // 0 não ativar rede 485,1 ativar rede 485
    end_cliente_num     =    1,                             // endereço cliente rede 485 
    adc_off_set_G       =    0,                             // off set conversão A/D  
    ident_sensor_G      =    0,                             // define endereço sensor 
    ciclos              =    0,                             // conta ciclos para reiniciar
    bit_0               =    0,
    bit_1               =    0,
    bit_2               =    0,
    bit_3               =    0,
    bit_A               =    0,
    bit_B               =    0,
    ciclos1             =    0;

float           results_G             = 0.00,               // armazena leitura conversor A/D
                results1_G            = 0.00,
                results2_G            = 0.00,
                results3_G            = 0.00,
                results4_G            = 0.00,
                results5_G            = 0.00,
                results6_G            = 0.00,
                results7_G            = 0.00,
                results8_G            = 0.00,
                values_G[num],                              // vetor de iteração de média móvel
                values1_G[num],
                values2_G[num],
                values3_G[num],
                values4_G[num],
                values5_G[num],
                values6_G[num],
                values7_G[num],
                values8_G[num],
                fator_calibracao_G   = 0.000;  
const int tam_msg = 62;                                     // tamanho buffer caracteres rede 485
char    charRecebida[tam_msg],                              // vetor de buffer rede 485
        str1[tam_msg];
String  str         = "",                                   // string de buffer de caracteres rede 485
        str_sensor  = "";
// =================================================================================
// --- Protótipo das Funções  organizados em outro arquivo---

// Monta média inicial.
extern  void partiu             (int conversor                                      );
// Chama média móvel da leitura do conversor de acordo com quantidade de sensores
extern  void media_leitura      (int conversor                                      );
// Realiza a média móvel de forma genérica
extern  float moving_average    (float sig_G, int identidade                        );
// Debug serial com dados brutos do conversor
extern  void imprimir_serial    (                                                   );
// Abre a rede 485 e envia dados.
extern  void Proto_485_V2       (String end_cliente, String msg_cliente             );
// Faz tratamento básico caso algum sensor não responda preenche vetor com zero.
extern  void Proto_485_V4       (char *testar, int tamanho                          );
//Coloca marcador de endereço no vetor de dados 
extern  void Proto_485_V3       (int ident_sensor_G                                 );
// função que faz a conversão do sinal analógico digitalizado em temperatura Graus
extern  float Conversor_mv_temp (float off_set_G, float f_calib_G, float results_G  );
//debug serial com temperatura convertida
extern void imprimir_serial_temp(                                                   );
// Realiza rotina de monitoramento da rede 485 esperando solicitação e rotina de leitura dos sensores
void Proto_485_V1               (int on_off                                         );
// envia solicitação na rede 485 para cliente responder dados, analisa ultimo cliente que enviou
extern void Gerenciador_rede           (                                                   );
// define endereço do micro baseado nos pinos de entrada baseado em binario e com marcador de quantos sensores online
extern void Gerenciador_endereco(                                                   );
// =================================================================================
// --- Configurações Iniciais ---
void setup() {
    wdt_disable     (               );                      // desativa o dog
    Serial.begin    (9600           );   
    delay           (100            );                      // parte porta serial debug                     
    Serial.println  ("CLIENTE-I2C-MULT-485-22-08-22-V4");
    Serial.flush    (               ); 
    mySerial.begin  (9600           );                      // parte porta serial debug
    pinMode         (rxPin,  INPUT  );                      // definir modo do pino Rx
    pinMode         (txPin,  OUTPUT );                      // definir modo do pino Tx
    pinMode         (MASTER, OUTPUT );                      // definir pino que controla rede 485
    digitalWrite    (MASTER, LOW    );                      // parte porta com nível lógico baixo.  
    wdt_enable      (WDTO_8S        );                      // ativa o wachdog  
    //delay(100);     
    /*  notas de configuração de wachdog e ganho dos conversores A/D
            #define WDTO_15MS,30MS,60MS,120MS,250MS,500MS,1S,2S,4S,8S 
            // partiu adc ads.setGain(GAIN_ONE      1x gain   +/- 4.096V  1 bit = 0.125mV
            //  GAIN_TWO   2x gain   +/- 2.048V  1 bit = 0.0625mV
            //  GAIN_FOUR  4x gain   +/- 1.024V  1 bit = 0.03125mV
            //  GAIN_EIGHT 8x gain   +/- 0.512V  1 bit = 0.015625mV
            //  GAIN_SIXTEEN 16x gain  +/- 0.256V  1 bit = 0.0078125mV  */    

    pinMode         (Pbit_0,  INPUT_PULLUP );  
    pinMode         (Pbit_1,  INPUT_PULLUP );  
    pinMode         (Pbit_2,  INPUT_PULLUP );  
    pinMode         (Pbit_3,  INPUT_PULLUP );  
    pinMode         (Pbit_A,  INPUT_PULLUP );  
    pinMode         (Pbit_B,  INPUT_PULLUP );  
   
    
    bit_0  = digitalRead(Pbit_0); // logica invertida para dar certo com montagem!
    bit_1  = digitalRead(Pbit_1); 
    bit_2  = digitalRead(Pbit_2); 
    bit_3  = digitalRead(Pbit_3);
    bit_A  = digitalRead(Pbit_A); 
    bit_B  = digitalRead(Pbit_B);
    // definindo endereço baseado nos pinos de config
    Gerenciador_endereco();
    int rest = end_cliente_num % 2;
    if (rest  ==  0)
    {
        conversor = 8;
        //Serial.println  (" rest com 8");
    }else{
        conversor = 1;
        //Serial.println  (" rest com 2");
        
    }
    
                      // 1 para um par; 8 para 8 pares 
    silencio            =     0;                            // 0 não ativar rede 485,1 ativar rede 485
    adc_off_set_G       =     0;   //42?                         // define off set conversor
    
    fator_calibracao_G  = 6.1810;                           // define fator de calibração
    if (conversor   >   0)    // PARTIDA DO CONVERSOR 1
    {
        ads_48.setGain  (GAIN_SIXTEEN);
       ads_48.begin    (0x48);
    }// END IF  
    
    if ( rest  ==  0 )  // PARTIDA DOS CONVERSORES 2 AO 4
    {
        Serial.println  (" ENDERECO PAR!");
        ads_49.setGain  (GAIN_SIXTEEN);  
        ads_49.begin    (0x49);
        
        ads_4A.setGain  (GAIN_SIXTEEN); 
        ads_4A.begin    (0x4A);
        
        ads_4B.setGain  (GAIN_SIXTEEN); 
        ads_4B.begin    (0x4B);
        // debug para indentificar a falta de um conversor
        // essa ideia deve ser melhorada!!!!
        float   asad = ads_48.readADC_Differential_2_3();
                Serial.println  ("ads_48");
                asad = ads_49.readADC_Differential_2_3();
                Serial.println  ("ads_49");
                asad = ads_4A.readADC_Differential_2_3();
                Serial.println  ("ads_4A");
                asad = ads_4B.readADC_Differential_2_3(); 
                Serial.println  ("ads_4B");
        
    }// END IF
    wdt_reset();
    if (conversor   >   0)    // DEBUG VIA 485 E SELECIONA ENDEREÇO CLIENTE
    {
        partiu          (conversor);
        if (rest != 0) Serial.println  (" com 2");
        if (rest == 0) Serial.println  (" com 8");
        Serial.print  ("\nM:");
        String end_cl = "",msg_c="";                          
        switch (end_cliente_num)
        { 
            case  1:
                        end_cl  = "@1"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl);  
                            break;
            case 2:
                        end_cl  = "@2"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 3:
                        end_cl  = "@3"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 4:
                        end_cl  = "@4"  ;    
                        Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 5:
                        end_cl  = "@5"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 6:
                        end_cl  = "@6"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 7:
                        end_cl  = "@7"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 8:
                        end_cl  = "@8"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 9:
                        end_cl  = "@9"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 10:
                        end_cl  = "@10"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 11:
                        end_cl  = "@11"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 12:
                        end_cl  = "@12"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 13:
                        end_cl  = "@13"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 14:
                        end_cl  = "@14"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            case 15:
                        end_cl  = "@15"  ;    
                            Proto_485_V2(end_cl,msg_c);
                            Serial.println  (end_cl); 
                            break;
            default:
                            break;
        }// end switch
    }// END IF
}// end setup
// =================================================================================
// --- Loop Infinito ---
// LEIA PARA CALIBRAR O EQUIPAMENTO
/**************************************
 * FAÇA LEITURA COM O RESISTOR CONHECIDO EXEMPLO   7 GRAUS 102,7OHM
 * FAÇA LEITURA COM O RESISTOR CONHECIDO EXEMPLO 123 GRAUS 147,2OHM
 * RECONHEÇA AS LEITURAS AD NOS DOIS PONTOS FAÇA A RELAÇÃO ENTRE
 * DELTA LEITURA AD / DELTA GRAUS O RESULTADO É O FATOR DE CALIBRAÇÃO
 * AJUSTE O OFF_SET DE ACORDO COM O COMPRIMENTO DO CABO
 *  
 * 
 * 
 ***************************************/
void loop() {
wdt_reset();
        if (ciclos1 > 10000) { // reinicia a cada 5 minutos ciclos se cont++ ativo
            ciclos1 = 0;
            wdt_disable     (               ); // desativa o dog
            wdt_enable      (WDTO_250MS     ); // ativa o wachdog
            Serial.println  ("R"            );
            delay           (1000           );
        }// end if
    ciclos1++;
    // BLOCO CALIBRAR
    //imprimir_serial();          //// Debug com valor bruto ADC ocupa 10% memória dinamica
    //imprimir_serial_temp();     //  / debug para ajuste de sistema MOSTRA TEMPERATURA CONVERTIDA
    //delay(1000);
    // BLOCO CALIBRAR
    int silencio = 1;                  // ativa rede 
    wdt_reset();

    // FUNÇÃO QUE LE PORTA SERIAL OU 485 E RESPONDE A DEMANDA
    Proto_485_V1(silencio);
    Gerenciador_rede();
    delay(100);

    // criar função que fica disparando chamadas para clientes cadastrados
    // função que envia comando para resposta de cliente
    if (Serial.available())
        {
             
        }else{

        }// end else

    // end
    
        if ((ciclos1%100)==0)
        {
            Serial.println  (ciclos1        );// a cada segundo
        }// end if
 
}// end loop
// =================================================================================

// =================================================================================
void Proto_485_V1(int on_off) {// refatorar essa função!!!!
    wdt_reset();
    String temp_msg_padrao = "";
    float t_m = 0.0;
    String temp_ = "";

    // rotina de leitura conversor A/D
        if (conversor >0)
        {
            ident_sensor_G = 1;
            results1_G = moving_average(ads_48.readADC_Differential_0_1(), ident_sensor_G);
            t_m = Conversor_mv_temp(0, fator_calibracao_G, results1_G            );
            // dividir por mil para colocar casas decimais
/************ DEBUG SERIAL!!! ***********************
            Serial.print  ("results1_G: ");
            Serial.println  (results1_G);
            Serial.print  ("t_m: ");
            Serial.println  (t_m);
            t_m = float(t_m / 1000);
            if (t_m<0) t_m = 0;
            Serial.print  ("convertido t_m: ");
            Serial.println  (t_m);
************  FIM DEBUG ***************************/
            t_m = float(t_m / 1000);
            if (t_m<0) t_m = 0;
             
            //if (t_m>98) t_m = 98;
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
            }
            if(conversor == 8){
                //Serial.print(" Rodou ate aqui | \n");
                ident_sensor_G =  3;
                results3_G = moving_average(ads_49.readADC_Differential_0_1(), ident_sensor_G);
                t_m = Conversor_mv_temp(adc_off_set_G, fator_calibracao_G, results3_G            );
                t_m = float(t_m / 1000);
                if (t_m<0) t_m = 0;
                if (t_m>98) t_m = 98;
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
                if (t_m>98) t_m = 98;
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
                if (t_m>98) t_m = 98;
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
                if (t_m>98) t_m = 98;
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
                if (t_m>98) t_m = 98;
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
                if (t_m>98) t_m = 98;
                if (t_m <10)temp_msg_padrao = String(t_m, 2);
                if (t_m>=10)temp_msg_padrao = String(t_m, 1);
                Proto_485_V3(ident_sensor_G);  
                temp_.concat(str_sensor); 
                temp_.concat(temp_msg_padrao);
            }// end if
            wdt_reset();
    // FIM rotina de leitura conversor A/D
    // criar testador de string para validar todas as posiçoes
    temp_.toCharArray(str1, temp_.length()+1);
    Proto_485_V4(str1,temp_.length()); // monta char com leituras 
    wdt_reset();
    // debug
    //Serial.print("\n");
    //Serial.print(str1);
    //Serial.println(" ");
    // fim debug
    /////////////////////////////////////////////
    // rotina de leitura porta 485 
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
/************ DEBUG SERIAL!!! ***********************
    // para rede 485
        /* while (mySerial.available()) {
            char request = mySerial.read();
            while (Serial.available()) {
            char request = Serial.read();
        */
/************  FIM DEBUG ***************************/
    while (mySerial.available()) {
            char request = mySerial.read();
            str.concat(request);
            int tamanho_string = str.length();
            charRecebida[conta_caracter] = request;
            conta_caracter++;
            wdt_reset();
        } // end while
        // debug
        //Serial.print("charRecebida: ");
        //Serial.print(charRecebida);
        //Serial.print(" | str:  ");
        //Serial.println(str);
        if (conta_caracter>3) {
                // debug
                //Serial.print("charRecebida: ");
                //Serial.print(charRecebida);
                //Serial.print(" | \n");
                //Serial.println(str);
                    String comparador = ""; //comparador.concat(charRecebida[i]);
                    int final_msg = 0, P_msg = 0;
                    for (size_t i = 0; charRecebida[i]!='\0'; i++)
                    {
                        if (charRecebida[i]=='P')   P_msg        = i;
                        if (charRecebida[i]=='*'){
                            final_msg   = i+1;
                            charRecebida[i+2]='\0';
                            conta_caracter = i+2;
                        }    
                    }// end for
                    if (final_msg !=0)
                    {
                        for (size_t i = 0; i < final_msg-P_msg; i++) {
                            comparador.concat(charRecebida[i]);
                            
                        }// end for
                    }// end if
                     wdt_reset();
                    // alterando método de resposta de chamada na rede
                    // agora quando receber o endereço do cliente o proprio cliente responte e nao o seguinte
                    // a logica anterior se chamar o p1 o p2 respondia
                    switch (end_cliente_num)
                    { 
                        case  2:
                                if (comparador == "P001*") end_cliente = "P002*";
                                break;
                        case  3:
                                if (comparador == "P002*") end_cliente = "P003*";
                                break;
                        case  4:
                                if (comparador == "P003*") end_cliente = "P004*";
                                break;
                        case  5:
                                if (comparador == "P004*") end_cliente = "P005*";
                                break;
                        case  6:
                                if (comparador == "P005*") end_cliente = "P006*";
                                break;
                        case  7:
                                if (comparador == "P006*") end_cliente = "P007*";
                                break;
                        case  8:
                                if (comparador == "P007*") end_cliente = "P008*";
                                break;
                        case  9:
                                if (comparador == "P008*") end_cliente = "P009*";
                                break;
                        case  10:
                                if (comparador == "P009*") end_cliente = "P010*";
                                break;
                        case  11:
                                if (comparador == "P010*") end_cliente = "P011*";
                                break;
                        case  12:
                                if (comparador == "P011*") end_cliente = "P012*";
                                break;
                        case  13:
                                if (comparador == "P012*") end_cliente = "P013*";
                                break;
                        case  14:
                                if (comparador == "P013*") end_cliente = "P014*";
                                break;
                        case  15:
                                if (comparador == "P014*") end_cliente = "P015*";
                                break;
                        case  1:
                                if (comparador == "P015*") end_cliente = "P001*";
                                break;
                        default:
                                wdt_reset();
                                break;
                    }// END SWITCH
                    wdt_reset();
                    if (end_cliente != "P00X*")
                    {
                        Proto_485_V2(end_cliente, msg_padrao);
                    }// end if
                    
                    ////limpa string e char mesmo sendo temporaria////////    
                    str.remove(0, str.length());
                    for (size_t i = 0; i < tam_msg-1; i++) {
                        charRecebida[i] = "";
                    }
                    digitalWrite(MASTER, LOW); // força a rede como cliente
                    ////limpa string e char mesmo sendo temporaria////////
                    //conta_caracter = 0;
                    if (conta_caracter>=3)
                    {
                        str.remove(0, str.length());
                        for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";
                        for (size_t i = 0;  str1[i]!='\0'; i++) str1[i] = "";
                        str = "";
                        digitalWrite(MASTER, LOW); // força a rede como cliente
                        ////limpa string e char mesmo sendo temporaria////////
                        conta_caracter = 0;
                        //Serial.print(" limpa string | \n");
                    }// end if
            } else {
                    if (conta_caracter>=3)
                    {
                        wdt_reset();
                        str.remove(0, str.length());
                        for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";
                        for (size_t i = 0;  str1[i]!='\0'; i++) str1[i] = "";
                        str = "";
                        digitalWrite(MASTER, LOW); // força a rede como cliente
                        ////limpa string e char mesmo sendo temporaria////////
                        conta_caracter = 0;
                        //Serial.print(" limpa string fora do if | \n");
                    }// end if   
                }// end else
    digitalWrite(MASTER, LOW);
    /////////////////////////////////////////////
} // end Proto_485_V1

// =================================================================================

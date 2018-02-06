//###########################################################################
/*
  SensorNode1: Las distintas funciones que tiene este código son:
  
              # Código para simular la lectura de datos de sensores.
              # Aplicar un algoritmo de compresión a los datos de sensores.
              # Transmitir un paquetes comprimidos a nodo receptor con muestras fijas.
*/
//###########################################################################

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//librerías.
#include <SPI.h>              
#include <LoRa.h>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//definiciones.
#define FAST_SENSORS                    1
#define SLOW_SENSORS                    2
#define MAX_PAYLOAD_SIZE                250
#define MAX_SAMPLES_COMPRESSED_AMOUNT1  6
#define MAX_SAMPLES_COMPRESSED_AMOUNT2  12
#define CAPTURE_TIME_SAMPLES            MAX_SAMPLES_COMPRESSED_AMOUNT1 * 1
#define COM_AND_TRANS_TIME_SAMPLES      1

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//definiciones de tipo.
typedef unsigned long ulong;
typedef const int     cint;

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//estructura para generación de árbol.
struct treeHuffmanCode
{
  //variables
  int di    = NULL;     //diferencia.
  int sum   = NULL;     //suma acumulativa de frecuencias.

  //punteros.
  treeHuffmanCode *left_tree  = NULL;
  treeHuffmanCode *right_tree = NULL;
  treeHuffmanCode *next_tree  = NULL;
};

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//funciones.
void  sendCompressData(byte _data[], byte _data_length, byte _data_type, byte _samples_compressed_amount, byte _destination_address);
void  initializeBitsResolutionData();
void joinBits(int _code, int _busy_bits_code, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);
void joinBits(int _code, int _busy_bits_code, int &_stream_com_data, int &_stream_available_bits);
void joinBits(int _code, int _busy_bits_code, int &_stream_com_data);
int compresorLDC(int *_data_in, int _data_in_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits);
void decompressorLDC(unsigned char *_stream_com_data, int _data_in_amount, int _bits_resolution_data, int *_data_out);
void setDi(int *_data_in, int *_di, int _data_in_amount);
void setFDI(int *_di, int _data_in_amount, int &_di_amount);
void orderFDI(void);
bool checkDiOnFdi(int _di, int _di_amount);
void iniTable(int *_data_in, int _data_in_amount, treeHuffmanCode &_tree_huffman_code, int &_fdi_amount);
void createTable(treeHuffmanCode &_node, int &_index_table);
void getHuffmanTree(treeHuffmanCode &_tree_huffman_code);
void getHuffmanCode(int _di, int &_code, int &_busy_bits_code);
int getDi(int _code, int _busy_bits_code);
int minor(int _a, int _b);
int  getLog2Di(int _di);
int maxTable(int _co);
void cleanIOBuffers1();
void  cleanCompressBuffers();
void  cleanReadBuffer();

//############################################################################
//############################################################################
//############################################################################
//############################################################################
//----------------- Variables globales para transmisor LoRa ------------------

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//pines para LoRa;
cint            csPin                         = 10;                           //selección de radio chip. 
cint            resetPin                      = 9;                            //radio reset.
cint            irqPin                        = 2;                            //pin de interrupción.                            

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//id de los paquetes.
int             data_id                       = 0;                                                          

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//dirección local.
byte            localAddress                  = 0xB1;

//############################################################################
//############################################################################
//############################################################################
//############################################################################
//--------------- Variables globales para compresor LEC ----------------------

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//cantidad de muestras a comprimir. 
/* 
  Parámetros modificables. Rango parámetros [2, MAX_SAMPLES_COMPRESSED_AMOUNT]
*/
byte            samples_compressed_amount1  = MAX_SAMPLES_COMPRESSED_AMOUNT1;
byte            samples_compressed_amount2  = MAX_SAMPLES_COMPRESSED_AMOUNT2;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//cantidad de bytes de encabezado personalizado.
/* 
  Parámetro modificable. El encabezado es definido dentro de la función "sendCompressData"
  específicamente en el buffer "_data", el cual es una referencia al buffer "stream_com_data"
*/
byte            header_bytes_amount         = 2;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//buffers y variables de lectura de datos.
byte            data_type                   = 0;                              //tipo de datos (rápido o lento).
byte            read_buffer[4];                                               //buffer auxiliar para leer puerto serial.
int             data_set1[9][MAX_SAMPLES_COMPRESSED_AMOUNT1];                 //buffer para sensores rápidos.
int             data_set2[5][MAX_SAMPLES_COMPRESSED_AMOUNT2];                 //buffer para sensores lentos.

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//buffers auxiliares.
int           di[MAX_SAMPLES_COMPRESSED_AMOUNT1];                      //diferencias entre xi y xi_1;
int           fdi[MAX_SAMPLES_COMPRESSED_AMOUNT1][2];                    //frecuencias de "di".
int           table[MAX_SAMPLES_COMPRESSED_AMOUNT1][3];                    //tabla de código Huffman.
int           fdi_amount          = -1;               //cantidad de "di".
treeHuffmanCode     tree_huffman_code;

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//buffers para resultados de la compresión.
int           table_bytes_amount      = 0;                //tamaño de la tabala en bytes.

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//buffers y variables para compresión de datos.
byte            stream_com_data[MAX_PAYLOAD_SIZE];                            //buffer para almacenar datos de entrada comprimidos.
int             stream_index_available      = header_bytes_amount;            //índice de buffer "stream_com_data" disponible para realizar concatenación.
int             stream_available_bits       = 8;                              //cantidad de bits disponibles para realizar concatenación en buffer "stream_com_data".
int             bits_resolution_data1[9];                                     //resolución en bits de los datos tipo 1 (rápidos).
byte            compress_size               = 0;                              //tamaño resultante de la compresión en bytes.
int             max_di1                     = 0;                              //máximo valor de di en tabla tipo 1.
int             max_code1                   = 0;                              //máximo valor de code en tabla tipo 1.
int             di_bits                     = 0;                              //bits para di.
int             code_bits                   = 0;                              //bits para code.

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//contador de muestras.
int             samples_counter1            = 0;
int             samples_counter2            = 0;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//tiempos.
int             capture_time_counter        = 0;                              //contador de tiempos de captura.
int             com_and_trans_time_counter  = 0;                              //contador de tiempos de compresión y transmisión.
unsigned long   intial_time;                                                  //tiempo de inicial.
unsigned long   capture_time[CAPTURE_TIME_SAMPLES];                           //tiempo de captura.
unsigned long   compress_time[COM_AND_TRANS_TIME_SAMPLES];                    //tiempo de compresión.
unsigned long   transmition_time[COM_AND_TRANS_TIME_SAMPLES];                 //tiempo de transmisión.
byte            capture_time_aux[CAPTURE_TIME_SAMPLES*4];                     //tiempo de captura auxiliar.
byte            compress_time_aux[COM_AND_TRANS_TIME_SAMPLES*4];              //tiempo de compresión auxiliar.
byte            transmition_time_aux[COM_AND_TRANS_TIME_SAMPLES*4];           //tiempo de transmisión auxiliar.
byte            aux_time_buffer[4];                                           //buffer de para tiempo auxiliar.

//############################################################################
//############################################################################
//############################################################################
//############################################################################
//------------------------------- Setup --------------------------------------
void setup() 
{
  //seteo de baudios para comunicación serial.
  Serial.begin(9600);                   
  Serial1.begin(9600);                  

  //sobreescritura de pines para LoRa (optional)
  LoRa.setPins(csPin, resetPin, irqPin);

  //bucle para inicializar LoRa.
  while (!LoRa.begin(915E6));

  //seteo de parámetros de configuración de LoRa
  LoRa.setTxPower(14,PA_OUTPUT_RFO_PIN);                                      //potencia.
  LoRa.setFrequency(866E6);                                                   //frecuencia de transmisión.
  LoRa.setSpreadingFactor(7);                                                 //Spread factor.
  LoRa.setSignalBandwidth(250E3);                                             //ancho de banda.  

  //limpieza de buffer de lectura.
  cleanReadBuffer();

  //inicialización de buffer de resoluciones de sensores.
  initializeBitsResolutionData();
}

//############################################################################
//############################################################################
//############################################################################
//############################################################################
//------------------------------- Loop ---------------------------------------
void loop()
{ 
  //consulta por cantidad de datos recibidos.
  if (Serial.available() > 0)
  {
    //lectura del tipo de datos enviado.
    data_type = Serial.read();

    //si es el tipo 1, entonces.
    if(data_type == 1)
    {     
      //limpieza de buffer de lectura.
      cleanReadBuffer();

      //captura de tiempo.
      intial_time = micros();
  
      //bucle para leer datos.
      for (int i1 = 0; i1 < 9; ++i1)
      {
        //captura del dato recibido.
        read_buffer[0] = Serial.read();
        read_buffer[1] = Serial.read();     
  
        //reconstrucción.
        data_set1[i1][samples_counter1] = *(int *)& read_buffer; 
      }  
      
      //captura de tiempo.      
      capture_time[capture_time_counter] = micros() - intial_time;

      //incremento de contador de tiempo de captura.
      ++capture_time_counter;
      
      //acuso de recibo de datos.
      Serial.write(1);     

      //incremento del contador de muestras.
      ++samples_counter1;

      //si se han obtenido todas la muestras necesarias para ser comprimidas, entonces.
      if(samples_counter1 == samples_compressed_amount1)
      { 
        //reset del contador de muestras.
        samples_counter1 = 0;
        
        //captura de tiempo.
        intial_time = micros();

        //bucle para comprimir muestras.
        for(int i1 = 0; i1 < 1;++i1)
        {
          //inicialización de compresor.
          iniTable(data_set1[i1], MAX_SAMPLES_COMPRESSED_AMOUNT2, tree_huffman_code, fdi_amount);
          
          //obtención del máximo de di y code en tabla.
          max_di1   = 10000;
          max_code1 = 5;
          
          //obtención de bits para di y code.
          di_bits   = getLog2Di(max_di1);
          code_bits = max_code1;
          
          //seteo de encabezado (largo tabla, largo di y largo code).
          joinBits(fdi_amount + 1, 8, stream_com_data, stream_index_available, stream_available_bits);
          joinBits(di_bits, 8, stream_com_data, stream_index_available, stream_available_bits);
          joinBits(code_bits, 8, stream_com_data, stream_index_available, stream_available_bits);
    
          //bucle para salvar la tabla.
          for (int i2 = 0; i2 < fdi_amount + 1; i2++)
          {
            //concatenación de bits para tabla.
            joinBits(table[i2][0], di_bits, stream_com_data, stream_index_available, stream_available_bits);        
            joinBits(table[i2][1], code_bits, stream_com_data, stream_index_available, stream_available_bits);
            joinBits(table[i2][2], 8, stream_com_data, stream_index_available, stream_available_bits);
          }

          //compresión de muestras.
          compress_size = compresorLDC(data_set1[i1], samples_compressed_amount1, bits_resolution_data1[i1], stream_com_data, stream_index_available, stream_available_bits); 
        }
        
        //captura de tiempo.      
        compress_time[com_and_trans_time_counter]     = micros() - intial_time;

        //captura de tiempo.
        intial_time                                   = micros();
        
        //transmisión de los datos comprimidos.
        sendCompressData(stream_com_data, compress_size, FAST_SENSORS, samples_compressed_amount1, 0xFF);

        //captura de tiempo.      
        transmition_time[com_and_trans_time_counter]  = micros() - intial_time;

        //incremento de contador de tiempo de compresión y transmisión.
        ++com_and_trans_time_counter;

        //si se han llenado los buffers de tiempos, entonces.
        if(com_and_trans_time_counter == COM_AND_TRANS_TIME_SAMPLES)
        {
          //bucle para obtener bytes.
          for (int i1 = 0; i1 < CAPTURE_TIME_SAMPLES; i1++) 
          {
            //obtención de bytes.
            *((int *)aux_time_buffer) = capture_time[i1];

            //seteo de bytes en buffer auxiliar.
            capture_time_aux[4*i1]      = aux_time_buffer[0];
            capture_time_aux[4*i1 + 1]  = aux_time_buffer[1];
            capture_time_aux[4*i1 + 2]  = aux_time_buffer[2];
            capture_time_aux[4*i1 + 3]  = aux_time_buffer[3];
          }
          
          //envío de tiempos de captura de datos a Source Gateway.
          Serial.write(capture_time_aux, CAPTURE_TIME_SAMPLES*4);

          //retardo.
          delay(1000);

          //bucle para obtener bytes.
          for (int i1 = 0; i1 < COM_AND_TRANS_TIME_SAMPLES; i1++) 
          {
            //obtención de bytes.
            *((int *)aux_time_buffer) = compress_time[i1];

            //seteo de bytes en buffer auxiliar.
            compress_time_aux[4*i1]      = aux_time_buffer[0];
            compress_time_aux[4*i1 + 1]  = aux_time_buffer[1];
            compress_time_aux[4*i1 + 2]  = aux_time_buffer[2];
            compress_time_aux[4*i1 + 3]  = aux_time_buffer[3];
          }

          //envío de tiempos de compresión de datos a Source Gateway.
          Serial.write(compress_time_aux, COM_AND_TRANS_TIME_SAMPLES*4);

          //retardo.
          delay(1000);

          //bucle para obtener bytes.
          for (int i1 = 0; i1 < COM_AND_TRANS_TIME_SAMPLES; i1++) 
          {
            //obtención de bytes.
            *((int *)aux_time_buffer) = transmition_time[i1];

            //seteo de bytes en buffer auxiliar.
            transmition_time_aux[4*i1]      = aux_time_buffer[0];
            transmition_time_aux[4*i1 + 1]  = aux_time_buffer[1];
            transmition_time_aux[4*i1 + 2]  = aux_time_buffer[2];
            transmition_time_aux[4*i1 + 3]  = aux_time_buffer[3];
          }

          //envío de tiempos de transmisión de datos a Source Gateway.
          Serial.write(transmition_time_aux, COM_AND_TRANS_TIME_SAMPLES*4);
        }        

        //limpieza de buffer y variables de compresión.
        cleanCompressBuffers();
      }
    }

    /*//si es el tipo 2, entonces.
    else if(data_type == 2)
    {  
      //limpieza de buffer de lectura.
      cleanReadBuffer();

      //captura de tiempo.
      intial_time = millis();
      
      //bucle para leer datos.
      for (int i1 = 0; i1 < 4; ++i1)
      {
        //captura del dato recibido.
        read_buffer[0] = Serial.read();
        read_buffer[1] = Serial.read();
  
        //reconstrucción.
        data_set2[i1][samples_counter2] = *(int *)& read_buffer;      
      }

      //captura del dato recibido.
      read_buffer[0] = Serial.read();
      read_buffer[1] = Serial.read();
      read_buffer[2] = Serial.read();
  
      //reconstrucción.
      data_set2[4][samples_counter2] = *(int *)& read_buffer; 

      //captura de tiempo.      
      //capture_time = intial_time - millis();
      
      //acuso de recibo de datos.
      Serial.write(2);

      //incremento del contador de muestras.
      ++samples_counter2;

      //si se han obtenido todas la muestras necesarias para ser comprimidas, entonces.
      if(samples_counter2 == samples_compressed_amount2)
      {
        //reset del contador de muestras.
        samples_counter2 = 0;

        //captura de tiempo.
        intial_time = millis();

        //compresión de las muestras.
        lec.compressorLEC(data_set2[0], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set2[1], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);   
        lec.compressorLEC(data_set2[2], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set2[3], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);

        //última compresión y obtención de la cantidad de bytes resultantes.
        compress_size = lec.compressorLEC(data_set2[4], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);

        //captura de tiempo.      
        //compress_time = intial_time - millis();

        //captura de tiempo.
        intial_time = millis();
        
        //transmisión de los datos comprimidos.
        sendCompressData(stream_com_data, compress_size, SLOW_SENSORS, samples_compressed_amount2, 0xFF);

        //captura de tiempo.      
        //transmition_time = intial_time - millis();

        //limpieza de buffer y variables de compresión.
        cleanCompressBuffers();
      }
    }*/
  }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para enviar paquetes de datos.
void sendCompressData(byte _data[], byte _data_length, byte _data_type, byte _samples_compressed_amount, byte _destination_address)
{
  //seteo de encabezado para descompresión.
  /*
    Para agregar más datos a encabezado, ud debe redefinir el parámetro "header_bytes_amount"
    de acuerdo a la cantidad de bytes que desea utilizar su encabezado (ej: header_bytes_amount = 3), 
    y luego setear el buffer "_data" en la posición que ud estime conveniente (ej: _data[1] = valor).
  */
  _data[0]  = _data_type;                                                     //tipo de datos comprimidos (sensores rápidos: 1, sensores lentos: 2).
  _data[1]  = _samples_compressed_amount;                                     //cantidad de muestras comprimidas.
  /*_data[2]  = capture_time;                                                   //tiempo de captura.
  _data[3]  = compress_time;                                                  //tiempo de compresión.
  _data[4]  = transmition_time;                                               //tiempo de trasnmisión.*/
  /*_data[3]= nuevo_datos;*/

  //creación del paquete a enviar.
  LoRa.beginPacket();                                                         //inicio del paquete.
  LoRa.write(_destination_address);                                           //dirección de destino.
  LoRa.write(localAddress);                                                   //dirección de fuente.
  LoRa.write(data_id);                                                        //id del paquete.
  LoRa.write(_data_length);                                                   //largo del payload.
  LoRa.write(_data, _data_length);                                            //payload.
  LoRa.endPacket();                                                           //finalización del paquete.

  //incremento del id del paquete.
  ++data_id;   
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para inicializar buffer de resolución de bits de datos de sensores.
void initializeBitsResolutionData()
{
  //--------------------------------------------------------------------------
  //seteo de resoluciones de datos rápidos.
  bits_resolution_data1[0] = 16;
  bits_resolution_data1[1] = 16;
  bits_resolution_data1[2] = 16;
  bits_resolution_data1[3] = 16;
  bits_resolution_data1[4] = 16;
  bits_resolution_data1[5] = 16;
  bits_resolution_data1[6] = 16;
  bits_resolution_data1[7] = 16;
  bits_resolution_data1[8] = 16;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para limpiar buffers y variables de compresión.
void cleanCompressBuffers()
{
  //limpieza de variables de compresión.
  stream_index_available  = header_bytes_amount;
  stream_available_bits   = 8;

  //bucle para limpiar buffer de datos comprimidos.
  for (int i1 = 0; i1 < MAX_PAYLOAD_SIZE; i1++)
  {
    //limpieza.
    stream_com_data[i1] = 0;
  }  
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//fución para limpiar buffer de recepción de datos.
void cleanReadBuffer()
{
  //limpieza de buffer de lectura.
  read_buffer[0] = 0;
  read_buffer[1] = 0;
  read_buffer[2] = 0;
  read_buffer[3] = 0;
}

//-------------------------------------------------------------------------------------
//función para concatenar bits en un arreglo.
void joinBits(int _code, int _busy_bits_code, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
  //--------------------------------------------------------------------------------- 
  //variables auxiliares.   
  unsigned char code_available_bits = 0;  
  unsigned char temp        = 0;
  unsigned char minor_or_equal    = 0;
  
  //si el buffer no puede contener a "_code".
  if (_stream_index_available == MAX_PAYLOAD_SIZE && _busy_bits_code > _stream_available_bits)
  {
    //impresión.
    printf("\n");
    printf("buffer lleno... \n");
    printf("\n");
  }

  //si el buffer puede contener a "_code".
  else
  {
    //inicialización de los bits disponibles para concatenar en "_code".
    code_available_bits = _busy_bits_code;
    
    //bucle para concatenar bits.  
    do
    {
      //obtención del menor número.
      minor_or_equal    = minor(_stream_available_bits, code_available_bits);
      
      //obtención de los bits de "_code" a concatenar. 
      temp        = (1 << minor_or_equal) - 1 & (_code >> (code_available_bits - minor_or_equal));
    
      //concatenación de bits de "_code" a "_stream_com_data".    
      _stream_com_data[_stream_index_available] = _stream_com_data[_stream_index_available] | (temp << (_stream_available_bits - minor_or_equal));
      
      //actualización de los bits disponibles en "_stream_com_data".
      _stream_available_bits  -= minor_or_equal;

      //actualización de la cantidad de bits disponibles en "_code".
      code_available_bits   -= minor_or_equal;

      //si se ha llenado la posición "_stream_index_available".
      if (_stream_available_bits == 0)
      {
        //incremento del índice de buffer "_stream_com_data" a utilizar.
        ++_stream_index_available;

        //reset de los bits disponibles en "_stream_com_data".
        _stream_available_bits =  8;
      }

    } while (code_available_bits > 0);
  }
}

//-------------------------------------------------------------------------------------
//función para concatenar bits en una variable.
void joinBits(int _code, int _busy_bits_code, int &_stream_com_data, int &_stream_available_bits)
{
  //si el buffer no puede contener a "_code".
  if (_busy_bits_code > _stream_available_bits)
  {
    //impresión.
    printf("\n");
    printf("buffer lleno... \n");   
    printf("\n");
  }

  //si el buffer puede contener a "_code".
  else
  {
    //concatenación de bits de "_code" a "_stream_com_data". 
    _stream_com_data = (_stream_com_data << _busy_bits_code) | (_code & ((1 << _busy_bits_code) - 1));

    //actualización de los bits disponibles en "_stream_com_data".
    _stream_available_bits -= _busy_bits_code;
  } 
}

//-------------------------------------------------------------------------------------
//función para concatenar bits en una variable.
void joinBits(int _code, int _busy_bits_code, int &_stream_com_data)
{
  //concatenación de bits de "_code" a "_stream_com_data". 
  _stream_com_data = (_stream_com_data << _busy_bits_code) | (_code & ((1 << _busy_bits_code) - 1));
}

//-------------------------------------------------------------------------------------
//función de compresión LDC.
int compresorLDC(int *_data_in, int _data_in_amount, int _bits_resolution_data, unsigned char *_stream_com_data, int &_stream_index_available, int &_stream_available_bits)
{
  //variables auxiliares.
  int code      = 0;
  int busy_bits_code  = 0;

  //bucle para obtener los "di".
  for (int i1 = 0; i1 < (_data_in_amount - 1); i1++)
  {
    //obtención de "di".
    di[i1] = _data_in[i1 + 1] - _data_in[i1];
  }

  //seteo de primer dato de entrada sin comprimir.
  joinBits(_data_in[0], _bits_resolution_data, _stream_com_data, _stream_index_available, _stream_available_bits);
  
  //bucle para obtener código huffman de cada "di".
  for (int i1 = 0; i1 < (_data_in_amount - 1); i1++)
  {
    //obtención del código.
    getHuffmanCode(di[i1], code, busy_bits_code);
    
    //seteo de código en buffer de datos comprimidos.
    joinBits(code, busy_bits_code, _stream_com_data, _stream_index_available, _stream_available_bits);
  }

  //retorno de la cantidad de bytes resultantes de la compresión.
  if (_stream_available_bits == 8)  return (_stream_index_available * 8);
  else                return (_stream_index_available * 8 + 8 - _stream_available_bits);
}

//-------------------------------------------------------------------------------------
//función de descompresión LDC.
void decompressorLDC(unsigned char *_stream_com_data, int _data_in_amount, int _bits_resolution_data, int *_data_out)
{
  //variables auxiliares. 
  int i1            = 0;
  int code          = 0;
  int di_aux          = 0;
  int di_index        = 0;
  int bit_position      = 0;
  int bits_counter      = 0;
  int index_amount      = int(floor((_bits_resolution_data - 8) / 8.0f));
  int remaining_bits_amount = 0;
  int temp          = 0;

  //seteo primeros 8 bits de "data_out[0]".
  joinBits(stream_com_data[0], 8, _data_out[0]);
  
  //bucle para setear los bytes restantes de "data_out[0]".
  for (i1 = 1; i1 < index_amount; i1++)
  {
    //seteo de bytes restantes de "data_out[0]".
    joinBits(_stream_com_data[i1], 8, _data_out[0]);
  }
  
  //bits remanentes de "data_out[0]".
  remaining_bits_amount = _bits_resolution_data - (8 * (index_amount + 1));
  
  //si existen bits restantes de "data_out[0]", entonces.
  if (remaining_bits_amount > 0)
  {
    //seteo de los bits restantes de "data_out[0]".
    temp  = stream_com_data[i1] >> (8 - remaining_bits_amount);
    joinBits(temp, remaining_bits_amount, _data_out[0]);
  }
  
  //actualización de variables auxiliares.
  bit_position = 7 - remaining_bits_amount; 

  //bucle para obtener "di".
  do
  {
    //obtención de "di".
    if ((_stream_com_data[i1] & (1 << bit_position)) == 0)  code = code << 1;
    else                          code = (code << 1) | 1;

    //decremento de "bit_position".
    --bit_position;

    //si "bit_position" alcanza su mínimo valor, entonces.
    if (bit_position == -1)
    {
      //actualizaciones.
      bit_position = 7;
      ++i1;
    }

    //incremento del contador de bits.
    ++bits_counter;

    //obtención de "di".
    di_aux = getDi(code, bits_counter);
    
    //si se ha obtenido "di", entonces.
    if (di_aux != -1)
    {
      //seteo de "di".
      di[di_index]  = di_aux;
      
      //incremento del índice de "di".
      ++di_index;

      //reset de variables.
      bits_counter  = 0;
      code      = 0;  

      //si es el último dato, entonces.
      if ((di_index + 1) == _data_in_amount) break;
    }

  } while (true);

  //bucle para obtener datos de salida.
  for (int i2 = 1; i2 < _data_in_amount; i2++)
  {
    //obtención de "data_out[i2]".
    _data_out[i2] = _data_out[i2 - 1] + di[i2 - 1];
  }
}

//-------------------------------------------------------------------------------------
//función para setear los "di".
void setDi(int *_data_in, int *_di, int _data_in_amount)
{
  //bucle para obtener los "di".
  for (int i1 = 0; i1 < (_data_in_amount - 1); i1++)
  {
    //obtención de "di".
    _di[i1] = _data_in[i1 + 1] - _data_in[i1];  
  }   
}

//-------------------------------------------------------------------------------------
//función para obtener la frecuencia ordenadas de menor a mayor de cada "di".
void setFDI(int *_di, int _data_in_amount, int &_di_amount)
{
  //bucle para cada "di".
  for (int i1 = 0; i1 < (_data_in_amount - 1); i1++)
  {
    //si "di" no está en buffer "fdi", entonces.
    if (!checkDiOnFdi(_di[i1], _di_amount))
    {
      //incremento del contador de "di".
      ++_di_amount;
      
      //seteo de "di" en "fdi".+
      fdi[_di_amount][0] = _di[i1];
      fdi[_di_amount][1] = 0;     

      //bucle para obtener la frecuencia de los "di".
      for (int i2 = 0; i2 < (_data_in_amount - 1); i2++)
      {
        //si "di[i1]" corresponde con "di[i2]". 
        if (_di[i1] == _di[i2])
        {
          //incremento de la frecuencia de "di".
          ++fdi[_di_amount][1];
        }
      }     
    }   
  }
  
  //ordenamiento de menor a mayor del buffer "fdi".
  orderFDI();
}

//-------------------------------------------------------------------------------------
//función para ordenadar las frecuencias de menor a mayor de cada "di".
void orderFDI(void)
{
  //variables auxiliares.
  int   temp_0    = 0;
  int   temp_1    = 0;
  bool  check_order = false;

  //ordenamiento de menor a mayor del buffer "fdi".
  do
  {
    //inicialización de variable auxiliar de checkeo de intercambio.
    check_order = false;

    //bucle para ordenar de menor a mayor buffer "fdi".
    for (int i1 = 0; i1 < fdi_amount; i1++)
    {
      //si el valor de "fdi" actual es mayor que el siguiente, entonces.
      if (fdi[i1][1] > fdi[i1 + 1][1])
      {
        //respaldo de valor actual.
        temp_0      = fdi[i1][0];
        temp_1      = fdi[i1][1];

        //intercambio de posiciones.
        fdi[i1][0]    = fdi[i1 + 1][0];
        fdi[i1][1]    = fdi[i1 + 1][1];
        fdi[i1 + 1][0]  = temp_0;
        fdi[i1 + 1][1]  = temp_1;

        //seteo de variable auxiliar de checkeo de intercambio.
        check_order = true;
      }
    }   

  } while (check_order);
}

//-------------------------------------------------------------------------------------
//función para comprobar si existe "di" en buffer "fdi".
bool checkDiOnFdi(int _di, int _di_amount)
{
  //bucle para checkear "di" en buffer "fdi".
  for (int i1 = 0; i1 < (_di_amount + 1); i1++)
  {
    //si existe, entonces retornar verdadero.
    if (_di == fdi[i1][0]) return true;
  }

  //si no existe, entonces retornar falso.
  return false;
}

//-------------------------------------------------------------------------------------
//función para inicializar tabla.
void iniTable(int *_data_in, int _data_in_amount, treeHuffmanCode &_tree_huffman_code, int &_fdi_amount)
{
  //índice para tabla.
  int index_table = 0;

  //reset de "_fdi_amount".
  _fdi_amount   = -1;

  //limpieza del árbol de Huffman.
  _tree_huffman_code.left_tree  = NULL;
  _tree_huffman_code.right_tree = NULL;
  _tree_huffman_code.next_tree  = NULL;

  /*//seteo de "fdi" central.
  fdi[HUFFMAN_RANGE][0] = 0;
  fdi[HUFFMAN_RANGE][1] = 10;

  //bucle para inicializar "fdi".
  for (int i1 = 0; i1 < HUFFMAN_RANGE; i1++)
  {
    //seteo de "di_aux".
    fdi[i1][0]            = HUFFMAN_RANGE - i1;
    fdi[i1][1]            = int(0.5 * i1 + 1);
    fdi[2 * HUFFMAN_RANGE - i1][0]  = -fdi[i1][0];
    fdi[2 * HUFFMAN_RANGE - i1][1]  = fdi[i1][1];
  } 

  //orden de menor a mayor del buffer "fdi".
  orderFDI();*/

  //seteo de los "di".
  setDi(_data_in, di, _data_in_amount);
  
  //seteo de "fdi" central.
  setFDI(di, _data_in_amount, _fdi_amount);
  
  //bucle para inicializar tabla.
  for (int i1 = 0; i1 < (_data_in_amount - 1); i1++)
  {
    //inicialización.
    table[i1][0] = 0;
    table[i1][1] = 0;
    table[i1][2] = 0;
  }

  //bucle para inicializar tabla.
  for (int i1 = 0; i1 < (_fdi_amount + 1); i1++)
  {
    //inicialización.
    table[i1][0] = fdi[i1][0];
  }

  //obtención del árbol para obtener código huffman.
  getHuffmanTree(tree_huffman_code);
  
  //creación de tabla con códigos huffman.
  createTable(_tree_huffman_code, index_table);
  
  //seteo del tamaño de la tabla en bytes.
  table_bytes_amount = index_table;
}

//-------------------------------------------------------------------------------------
//función para crear tabla de códigos huffman.
void createTable(treeHuffmanCode &_node, int &_index_table)
{
  //código auxiliar.
  int aux_code  = table[_index_table][1];
  int counter   = table[_index_table][2];

  //si "fdi_amount" es cero, entonces.
  if (fdi_amount == 0)
  {
    //seteo de "di" y "0" en código.
    table[_index_table][0] = _node.di;
    table[_index_table][1] = 0;
    table[_index_table][2] = 1;
  }

  //si "fdi_amount" es distinto de cero, entonces.
  else
  {
    //si se ha llegado a un "di" en la izquierda, entonces.
    if (_node.left_tree->left_tree == NULL)
    {
      //seteo de "di" y "0" en código.
      table[_index_table][0] = _node.left_tree->di;
      table[_index_table][1] = aux_code << 1;
      table[_index_table][2] = counter + 1;

      //incremento del índice de tabla.
      ++_index_table;
    }

    //si no se ha llegado a un "di" en la izquierda, entonces.
    else
    {
      //seteo de "0" en código.
      table[_index_table][1] = aux_code << 1;
      table[_index_table][2] = counter + 1;

      //se continúa la creación de la tabla.
      createTable(*_node.left_tree, _index_table);
    }
  
    //si se ha llegado a un "di" en la derecha, entonces.
    if (_node.right_tree->right_tree == NULL)
    {
      //seteo de "di" y "1" en código.
      table[_index_table][0] = _node.right_tree->di;
      table[_index_table][1] = (aux_code << 1) | 1;
      table[_index_table][2] = counter + 1;

      //incremento del índice de tabla.
      ++_index_table;
    }

    //si no se ha llegado a un "di" en la derecha, entonces.
    else
    {
      //seteo de "1" en código.
      table[_index_table][1] = (aux_code << 1) | 1;
      table[_index_table][2] = counter + 1;

      //se continúa la creación de la tabla.
      createTable(*_node.right_tree, _index_table);
    }
  } 
}

//-------------------------------------------------------------------------------------
//función para obtener árbol de código huffman tipo 1.
void getHuffmanTree(treeHuffmanCode &_tree_huffman_code)
{
  //variables auxiliares.
  int       tree_amount   = fdi_amount + 1;
  treeHuffmanCode *new_tree   = NULL;
  treeHuffmanCode *tree_root    = NULL; 
  treeHuffmanCode *tree_aux   = NULL;
  treeHuffmanCode *tree_pres_aux  = NULL;
  treeHuffmanCode *tree_past_aux  = NULL;
  
  //creación inicial de árboles con "fdi".
  for (int i1 = 0; i1 < tree_amount; i1++)
  {
    //creación de árboles.
    new_tree  = (treeHuffmanCode*)(malloc(sizeof(treeHuffmanCode)));

    //si "i1" es igual que "cero" entonces.
    if (i1 == 0)  tree_root     = new_tree;
    else      tree_aux->next_tree = new_tree;

    //seteo de puntero auxiliar.
    tree_aux        = new_tree;

    //seteo del árbol creado.
    new_tree->di      = fdi[i1][0];
    new_tree->sum     = fdi[i1][1];
    new_tree->left_tree   = NULL;
    new_tree->right_tree  = NULL;   
    new_tree->next_tree   = NULL;     
  }

  //si hay más de un árbol, entonces.
  if (tree_amount > 1)
  {
    //bucle para setear árbol para obtener código huffman.
    do
    {
      //creación de un nodo con los dos primeros árboles.
      new_tree        = (treeHuffmanCode*)(malloc(sizeof(treeHuffmanCode)));
  
      //seteo del árbol creado.
      new_tree->sum     = tree_root->sum + tree_root->next_tree->sum;
      new_tree->left_tree   = tree_root;
      new_tree->right_tree  = tree_root->next_tree;
      new_tree->next_tree   = NULL;   

      //actualización de la cantidad de árboles.
      --tree_amount;    

      //si hay un solo árbol, entonces.
      if (tree_amount == 1) break;

      //si hay más de un árbol, entonces.
      else 
      {
        //seteo de punteros auxiliares.
        tree_pres_aux = tree_root->next_tree->next_tree;
        tree_past_aux = tree_pres_aux;  

        //bucle para concatenar nuevo nodo.
        do
        {
          //si la suma acumulada de frecuencias del nuevo nodo es menor, entonces.
          if (new_tree->sum < tree_pres_aux->sum)
          {
            //concatenación del nuevo nodo con el siguiente.
            new_tree->next_tree = tree_pres_aux;

            //si los punteros son distintos.
            if (tree_pres_aux != tree_past_aux)
            {
              //concatenación del nuevo nodo con el anterior
              tree_past_aux->next_tree = new_tree;

              //actualización del árbol raiz.
              tree_root = tree_root->next_tree->next_tree;
            }

            //si los punteros son iguales.
            else
            {
              //actualización del árbol raiz.
              tree_root = new_tree;
            }

            //salir del bucle.
            break;
          }

          //si la suma acumulada de frecuencias del nuevo nodo es igual, entonces.
          else if (new_tree->sum == tree_pres_aux->sum)
          {
            //concatenación del nuevo nodo con el siguiente y el anterior.
            new_tree->next_tree     = tree_pres_aux->next_tree;
            tree_pres_aux->next_tree  = new_tree;         

            //actualización del árbol raiz.
            tree_root         = tree_root->next_tree->next_tree;

            //salir del bucle.
            break;
          }

          //si la suma acumulada de frecuencias del nuevo nodo es mayor, entonces.
          else if (new_tree->sum > tree_pres_aux->sum)
          {
            //actualización de punteros auxiliares.
            tree_past_aux = tree_pres_aux;
            tree_pres_aux = tree_pres_aux->next_tree;

            //si se ha llegado al último nodo, entonces.
            if (tree_pres_aux == NULL)
            {
              //concatenación del nuevo nodo con el siguiente.
              tree_past_aux->next_tree  = new_tree;
              new_tree->next_tree     = NULL;

              //actualización del árbol raiz.
              tree_root         = tree_root->next_tree->next_tree;

              //salir del bucle.
              break;
            }
          }

        } while (true);

        //reset de punteros.
        new_tree->left_tree->next_tree  = NULL;
        new_tree->right_tree->next_tree = NULL; 
      }

    } while (true);
  } 

  //seteo de la raiz del árbol creado.
  _tree_huffman_code.di     = new_tree->di;
  _tree_huffman_code.next_tree  = new_tree->next_tree;
  _tree_huffman_code.left_tree  = new_tree->left_tree;
  _tree_huffman_code.right_tree = new_tree->right_tree;
}

//-------------------------------------------------------------------------------------
//función para obtener "huffman code" en tabla.
void getHuffmanCode(int _di, int &_code, int &_busy_bits_code)
{
  //bucle para obtener código huffman de tabla.
  for (int i1 = 0; i1 < (fdi_amount + 1); i1++)
  {
    //si existe correspondencia de "_di", entonces.
    if (_di == table[i1][0])
    {

      //seteo de código y bits ocupados por el código.
      _code     = table[i1][1];
      _busy_bits_code = table[i1][2];
    }
  }
}

//-------------------------------------------------------------------------------------
//función para obtener "di" en tabla.
int getDi(int _code, int _busy_bits_code)
{
  //bucle para cotejar "_code" en tabla.
  for (int i1 = 0; i1 < (fdi_amount + 1); i1++)
  {
    //si existe en tabla, entonces.
    if (_code == table[i1][1] && _busy_bits_code == table[i1][2]) return table[i1][0];
  }

  //si no existe en tabla, entonces.
  return -1;
}

//-------------------------------------------------------------------------------------
//función que devuelve el valor menor entre "_a" y  "_b".
int minor(int _a, int _b)
{
  //retorno del valor menor o igual.
  if (_a <= _b) return _a;
  return _b;
}

//-------------------------------------------------------------------------------------
//función para calcular log2 de "di".
int getLog2Di(int _di)
{
  //auxiliar data.
  int aux_di  = _di;
  int ni    = 0;

  //bucle para calcular log2 de di.
  while (aux_di > 0)
  {
    //calculo de log2 de di.
    aux_di = aux_di / 2;
    ++ni;
  }

  //retorno de ni.
  return ni;
}

//-------------------------------------------------------------------------------------
//método para obtener el máximo valor de una columna de la tabla.
int maxTable(int _co)
{
  //inicialización del valor máximo.
  int maxv = table[0][_co];

  //bucle para hallar valor máximo.
  for (int i1 = 1; i1 < fdi_amount + 1; i1++)
  {
    //actualización del valor máximo.
    if (maxv < table[i1][_co]) maxv = table[i1][_co];
  }

  //retorno del valor máximo.
  return maxv;
}

//-------------------------------------------------------------------------------------
//función para limpiar buffer de datos de entrada, salida y diferencias tipo 1.
void cleanIOBuffers1()
{
  //bucle para limpiar buffers.
  for (int i1 = 0; i1 < MAX_SAMPLES_COMPRESSED_AMOUNT1; i1++)
  {
    //limpieza.
    di[i1]      = 0;
    fdi[i1][0]    = -1;
    fdi[i1][1]    = -1;
    table[i1][0]  = -1;
    table[i1][1]  = -1;
    table[i1][2]  = -1;
  }
}




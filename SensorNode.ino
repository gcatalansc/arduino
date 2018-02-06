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
#include <compressor_lec.h>
//#include <compressor_sn.h>

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//definiciones.
#define FAST_SENSORS                    1
#define SLOW_SENSORS                    2
#define MAX_PAYLOAD_SIZE                250
#define MAX_SAMPLES_COMPRESSED_AMOUNT1  6
#define MAX_SAMPLES_COMPRESSED_AMOUNT2  12

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//definiciones de tipo.
typedef unsigned long ulong;
typedef const int     cint;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//funciones.
void sendCompressData(byte _data[], byte _data_length, byte _data_type, byte _samples_compressed_amount, byte _destination_address);
void cleanCompressBuffers();
void cleanReadBuffer();

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
//compresor de datos LEC.
compressor_lec  lec;

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

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//buffers y variables para compresión de datos.
byte            stream_com_data[MAX_PAYLOAD_SIZE];                            //buffer para almacenar datos de entrada comprimidos.
int             stream_index_available      = header_bytes_amount;            //índice de buffer "stream_com_data" disponible para realizar concatenación.
int             stream_available_bits       = 8;                              //cantidad de bits disponibles para realizar concatenación en buffer "stream_com_data".
int             bits_resolution_data;                                         //resolución en bits de los datos.
byte            compress_size               = 0;                              //tamaño resultante de la compresión en bytes.

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//contador de muestras.
int             samples_counter1            = 0;
int             samples_counter2            = 0;

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

  //inicialización de compresor LEC.
  lec.initialization(MAX_SAMPLES_COMPRESSED_AMOUNT2, MAX_PAYLOAD_SIZE);

  //limpieza de buffer de lectura.
  cleanReadBuffer();
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
  
      //bucle para leer datos.
      for (int i1 = 0; i1 < 9; ++i1)
      {
        //captura del dato recibido.
        read_buffer[0] = Serial.read();
        read_buffer[1] = Serial.read();     
  
        //reconstrucción.
        data_set1[i1][samples_counter1] = *(int *)& read_buffer; 
      }  

      //acuso de recibo de datos.
      Serial.write(1);     

      //incremento del contador de muestras.
      ++samples_counter1;

      //si se han obtenido todas la muestras necesarias para ser comprimidas, entonces.
      if(samples_counter1 == samples_compressed_amount1)
      {
        //reset del contador de muestras.
        samples_counter1 = 0;
        
        //compresión de las muestras.
        lec.compressorLEC(data_set1[0], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);        
        lec.compressorLEC(data_set1[1], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);       
        lec.compressorLEC(data_set1[2], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set1[3], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set1[4], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set1[5], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set1[6], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set1[7], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);

        //última compresión y obtención de la cantidad de bytes resultantes.
        compress_size = lec.compressorLEC(data_set1[8], samples_compressed_amount1, stream_com_data, stream_index_available, stream_available_bits);

        //transmisión de los datos comprimidos.
        sendCompressData(stream_com_data, compress_size, FAST_SENSORS, samples_compressed_amount1, 0xFF);

        //limpieza de buffer y variables de compresión.
        cleanCompressBuffers();
      }
    }

    //si es el tipo 2, entonces.
    else if(data_type == 2)
    {  
      //limpieza de buffer de lectura.
      cleanReadBuffer();
      
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
      
      //acuso de recibo de datos.
      Serial.write(2);

      //incremento del contador de muestras.
      ++samples_counter2;

      //si se han obtenido todas la muestras necesarias para ser comprimidas, entonces.
      if(samples_counter2 == samples_compressed_amount2)
      {
        //reset del contador de muestras.
        samples_counter2 = 0;

        //compresión de las muestras.
        lec.compressorLEC(data_set2[0], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set2[1], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);   
        lec.compressorLEC(data_set2[2], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);
        lec.compressorLEC(data_set2[3], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);

        //última compresión y obtención de la cantidad de bytes resultantes.
        compress_size = lec.compressorLEC(data_set2[4], samples_compressed_amount2, stream_com_data, stream_index_available, stream_available_bits);

        //transmisión de los datos comprimidos.
        sendCompressData(stream_com_data, compress_size, SLOW_SENSORS, samples_compressed_amount2, 0xFF);

        //limpieza de buffer y variables de compresión.
        cleanCompressBuffers();
      }
    }
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


//#########################################################################################################################


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para enviar paquetes de datos.
void sendCompressDataX(int _data1, int _data2, int _data3, int _data4)
{ 
  //creación del paquete a enviar.
  LoRa.beginPacket();                                                         //inicio del paquete.
  LoRa.write(0xFF);                                           //dirección de destino.
  LoRa.write(0x81);                                                   //dirección de fuente.
  LoRa.write(data_id);                                                        //id del paquete.
  LoRa.write(6);                                                   //largo del payload.
  LoRa.write(5);                                            //payload.
  LoRa.write(5);                                            //payload.
  LoRa.write(_data1);                                            //payload.
  LoRa.write(_data2);                                            //payload.
  LoRa.write(_data3);                                            //payload.
  LoRa.write(_data4);                                            //payload.
  LoRa.endPacket();                                                           //finalización del paquete.

  //incremento del id del paquete.
  ++data_id;   
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para enviar paquetes de datos.
void sendCompressDataXX(int _data1)
{ 
  //creación del paquete a enviar.
  LoRa.beginPacket();                                                         //inicio del paquete.
  LoRa.write(0xFF);                                           //dirección de destino.
  LoRa.write(0x81);                                                   //dirección de fuente.
  LoRa.write(data_id);                                                        //id del paquete.
  LoRa.write(3);                                                   //largo del payload.
  LoRa.write(5);                                            //payload.
  LoRa.write(5);                                            //payload.
  LoRa.write(_data1);                                            //payload.
  LoRa.endPacket();                                                           //finalización del paquete.

  //incremento del id del paquete.
  ++data_id;   
}




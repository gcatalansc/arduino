#ifndef __TF_DEFINITIONS_H__
#define __TF_DEFINITIONS_H__

//------------------------------------------------
//librerías.
#include <SPI.h>
#include <SD.h>

//definiciones de nombre para compresores.
#define   ALDC      0
#define   FELACS    1
#define   LDC       2
#define   LEC       3
#define   SLZW_MC   4

//buffer size of stream compress data.  
#define   MAX_BUFFER_SIZE   100
#define   LENGTH_DATASET    32

//------------------------------------------------
//definiciones.
#define   DS1307_I2C_ADDRESS 0x68

//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//DEFINICIONES PARA TRATAMIENTO DE DATOS DE SENSORES.

//data settings.
const int     bits_resolution_data    = 1;                 //resolución en bits de los datos.
const int     samples_amount          = 1;                 //cantidad de muestras tomadas.
const int     iterations_amount       = 3;                 //iteraciones por algoritmo.

//buffers auxiliares.
unsigned char stream_com_data[MAX_BUFFER_SIZE];            //buffer para almacenar datos de entrada comprimidos.
int           stream_index_available  = 0;                 //índice de buffer "stream_com_data" disponible para realizar concatenación.
int           stream_available_bits   = 8;                 //cantidad de bits disponibles para realizar concatenación en buffer "stream_com_data".

//input data and output data.
int           data_in[samples_amount];                     //datos de entrada del compresor por iteración.
int           data_out[samples_amount];                    //datos de salida del descompresor.

//resultados del benchmark.
float         CR[4];

//fichero de set de datos de prueba y buffer auxiliar.
int           dataset[LENGTH_DATASET];
int           dataset_index           = 0;

//tipo de algoritmo escogido para comprimir.
int           algorithm_counter       = 0;

//cantidades de bytes resultantes de la compresión y descompresión.
int           compress_size           = 0;
int           decompress_size         = 0;

//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//DEFINICIONES PARA ENVÍO Y RECEPCIÓN DE DATOS.

//------------------------------------------------
//parámetros de buffers de envío y recepción.
const int delay_time                        = 5000;                     //tiempo de espera por iteración.
const int static_frame_bytes_length         = 15;                       //largo estático del frame de envío de datos.
byte      received_frame[static_frame_bytes_length + MAX_BUFFER_SIZE];  //buffer de recepción de datos.
byte      send_frame[static_frame_bytes_length + MAX_BUFFER_SIZE];      //buffer de envío de datos.

//------------------------------------------------
//estructura del frame recibido.
byte      start_delimiter_r;
byte      frame_length_r[2];
byte      frame_type_r;
byte      address_r[8];
byte      rssi_r;
byte      options_r;
byte      rf_data_r[241];
byte      checksum_r;

//------------------------------------------------
//banderas y variables auxiliares.
bool      reception_estatus     = false;
int       received_frame_length = 0;
int       counter_serial_data   = 0;

//------------------------------------------------
//parámetros para la tarjeta SD.
const int chipSelect            = 8;

//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//DECLARACIÓN DE FUNCIONES.

//------------------------------------------------
//función para enviar frame.
void sendDataFrame(unsigned char *_stream_com_data, int _compress_size);

//------------------------------------------------
//función para recibir frame.
void receptionDataFrame();

//------------------------------------------------
//función para setear estructura de frame recibido.
void setReceivedData();

//------------------------------------------------
//función para guardar datos de los sensores en SD.
void saveDataInSD(int *_data_in, int _samples_amount);

//------------------------------------------------
//función para imprimir recepción de datos.
void printReceivedData();

//------------------------------------------------
//función para imprimir mediciones de datos.
void printDataOfSensors(int *_data_in, int _samples_amount);

//------------------------------------------------
//función para obtener checksum.
byte getChecksum(byte _frame[], int _frame_length);

//------------------------------------------------
//función para inicializar buffer de envío y recepción.
void initializedBufferToSendAndReceptionData();

//------------------------------------------------
//función para inicializar "dataloger".
void initializedDataloger();

//------------------------------------------------
//función para leer buffer de set de datos de prueba.
void redBufferDataSet(int *_data_in, int _samples_amount, int *_dataset, int &_dataset_index);

//-------------------------------------------------------------------------------------
//función para leer fichero de set de datos de prueba.
void readFileDataSet(char *_filename, int *_dataset, int _length_dataset);

//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//------------------------------------------------
//DEFINICIÓN DE FUNCIONES.

//------------------------------------------------
//función para enviar frame.
void sendDataFrame(unsigned char *_stream_com_data, int _compress_size)
{
 //frame id.     
 send_frame[4] += 0x01; 

 //bucle para cargar datos comprimidos.
 for (int i1 = 0; i1 < _compress_size; i1++)
 {
   //carga de datos compromidos.
   send_frame[static_frame_bytes_length + i1 - 1] = _stream_com_data[i1];
 }

 //checksum.
 send_frame[static_frame_bytes_length + _compress_size - 1] = getChecksum(send_frame, static_frame_bytes_length + _compress_size);

 //envío de frame.
 Serial1.write(send_frame,static_frame_bytes_length + _compress_size);
 
 //limpieza del buffer serial.
 //Serial1.flush();
}

//------------------------------------------------
//función para recibir frame.
void receptionDataFrame()
{  
  //variable auxiliar para dato de cabecera.
  byte read_buffer = 0;
  
  //si se han recibido datos, entonces.
  if(Serial1.available() > 0)
  {   
    //captura del dato recibido.
    read_buffer = Serial1.read();
    
    //si es el comienzo de un frame entonces.
    if(read_buffer == 0x7E && reception_estatus == false)
    {      
      //actualización de la bandera de estado de recepción.
      reception_estatus = true; 
    
      //inicialización del contador de datos leidos.
      counter_serial_data = 1;
            
      //si existen más datos recibidos, entonces.
      if(Serial1.available() > 0)
      {
        //incremento del contador de datos leidos.
        ++counter_serial_data;
            
        //obtención del largo del frame[0].
        frame_length_r[0]  = Serial1.read();
        received_frame[1]  = frame_length_r[0];

        //si existen más datos recibidos, entonces.
        if(Serial1.available() > 0)
        {
          //incremento del contador de datos leidos.
          ++counter_serial_data;
            
          //obtención del largo del frame[1].
          frame_length_r[1]     = Serial1.read();
          received_frame[2]     = frame_length_r[1];

          //cálculo del largo del frame.
          received_frame_length = int(frame_length_r[0])*100 + int(frame_length_r[1]);
          
          //si el frame recibido no corresponde a un acuso de recibo.
          if(received_frame_length != 3)
          {
            //bucle para leer datos.
            while(Serial1.available() > 0 && (counter_serial_data <= received_frame_length + 4))
            {            
              //captura de los datos recibidos.
              received_frame[counter_serial_data] = Serial1.read();  
  
              //incremento del contador de datos leidos.
              ++counter_serial_data;

              if(counter_serial_data == received_frame_length + 4)
              {
                //actualización de la bandera de estado de recepción.
                reception_estatus = false; 

                //seteo de estructura de frame recibido.
                setReceivedData();
              }
            }
          }

          //si el frame recibido corresponde a un acuso de recibo.
          else if(received_frame_length == 3)
          {
            //bucle para leer datos.
            while(Serial1.available() > 0 && counter_serial_data < 7)
            {
              //lectura del puerto serial.
              Serial1.read(); 

              //incremento del contador de datos leidos.
              ++counter_serial_data;

              if(counter_serial_data == 7)
              {
                //actualización de la bandera de estado de recepción.
                reception_estatus = false; 
              }
            }
          }
        } 

        //si no existen más datos recibidos, entonces.
        else return;
      }

      //si no existen más datos recibidos, entonces.
      else return;
    }

    //si no es el comienzo de un frame entonces.
    else
    {
      //si se ha leído solo la cabecera del frame, entonces.
      if(counter_serial_data == 1)
      {
        //incremento del contador de datos leidos.
        ++counter_serial_data;
            
        //obtención del largo del frame[0].
        frame_length_r[0]  = read_buffer;
        received_frame[1]  = frame_length_r[0];

        //si existen más datos recibidos, entonces.
        if(Serial1.available() > 0)
        {
          //incremento del contador de datos leidos.
          ++counter_serial_data;
            
          //obtención del largo del frame[1].
          frame_length_r[1]     = Serial1.read();
          received_frame[2]     = frame_length_r[1];

          //cálculo del largo del frame.
          received_frame_length = int(frame_length_r[0])*100 + int(frame_length_r[1]);

          //si el frame recibido no corresponde a un acuso de recibo.
          if(received_frame_length != 3)
          {
            //bucle para leer datos.
            while(Serial1.available() > 0 && (counter_serial_data <= received_frame_length + 4))
            {
              //captura de los datos recibidos.
              received_frame[counter_serial_data] = Serial1.read();  
  
              //incremento del contador de datos leidos.
              ++counter_serial_data;   

              if(counter_serial_data == received_frame_length + 4)
              {
                //actualización de la bandera de estado de recepción.
                reception_estatus = false; 

                //seteo de estructura de frame recibido.
                setReceivedData();
              }
            }
          }

          //si el frame recibido corresponde a un acuso de recibo.
          else if(received_frame_length == 3)
          {
            //bucle para leer datos.
            while(Serial1.available() > 0 && counter_serial_data < 7)
            {
              //lectura del puerto serial.
              Serial1.read(); 

              //incremento del contador de datos leidos.
              ++counter_serial_data;

              if(counter_serial_data == 7)
              {
                //actualización de la bandera de estado de recepción.
                reception_estatus = false; 
              }
            }
          }
        } 

        //si no existen más datos recibidos, entonces.
        else return;
      }

      //si se ha leído solo un byte del largo del frame, entonces.
      else if(counter_serial_data == 2)
      {
        //incremento del contador de datos leidos.
        ++counter_serial_data;
            
        //obtención del largo del frame[1].
        frame_length_r[1]     = read_buffer;
        received_frame[2]     = frame_length_r[1];

        //cálculo del largo del frame.
        received_frame_length = int(frame_length_r[0])*100 + int(frame_length_r[1]);

        //si el frame recibido no corresponde a un acuso de recibo.
        if(received_frame_length != 3)
        {
          //bucle para leer datos.
          while(Serial1.available() > 0 && (counter_serial_data <= received_frame_length + 4))
          {
            //captura de los datos recibidos.
              received_frame[counter_serial_data] = Serial1.read();  
  
            //incremento del contador de datos leidos.
            ++counter_serial_data;        

            if(counter_serial_data == received_frame_length + 4)
            {
              //actualización de la bandera de estado de recepción.
              reception_estatus = false; 

              //seteo de estructura de frame recibido.
              setReceivedData();
            }
          }
        }

        //si el frame recibido corresponde a un acuso de recibo.
        else if(received_frame_length == 3)
        {
          //bucle para leer datos.
          while(Serial1.available() > 0 && counter_serial_data < 7)
          {
            //lectura del puerto serial.
            Serial1.read(); 

            //incremento del contador de datos leidos.
            ++counter_serial_data;

            if(counter_serial_data == 7)
            {
              //actualización de la bandera de estado de recepción.
              reception_estatus = false; 
            }
          }
        }
      }

      //si ya se ha leído el largo del frame.
      else if(counter_serial_data    >   2)
      {
        //si el frame recibido no corresponde a un acuso de recibo.
        if(received_frame_length != 3)
        {
          //captura de los datos recibidos.
          received_frame[counter_serial_data] = read_buffer;
          
          //incremento del contador de datos leidos.
          ++counter_serial_data;
        
          //bucle para leer datos.
          while(Serial1.available() > 0 && (counter_serial_data <= received_frame_length + 4));
          {
            //captura de los datos recibidos.
            received_frame[counter_serial_data] = Serial1.read();  

            //incremento del contador de datos leidos.
            ++counter_serial_data;  

            if(counter_serial_data == received_frame_length + 4)
            {
              //actualización de la bandera de estado de recepción.
              reception_estatus = false; 

              //seteo de estructura de frame recibido.
              setReceivedData();
            }
          }
        }  

        //si el frame recibido corresponde a un acuso de recibo.
        else if(received_frame_length == 3)
        {
          //bucle para leer datos.
          while(Serial1.available() > 0 && counter_serial_data < 7)
          {
            //lectura del puerto serial.
            Serial1.read(); 
  
            //incremento del contador de datos leidos.
            ++counter_serial_data;
  
            if(counter_serial_data == 7)
            {
              //actualización de la bandera de estado de recepción.
              reception_estatus = false; 
            }
          }
        }
      }
    }
  }  
}

//------------------------------------------------
//función para setear estructura de frame recibido.
void setReceivedData()
{
  //reset del contador de datos leidos.
  counter_serial_data = 0;

  //frame type.
  frame_type_r       = received_frame[3];
      
  //64 bit address of destination.
  address_r[0]       = received_frame[4];
  address_r[1]       = received_frame[5];
  address_r[2]       = received_frame[6];
  address_r[3]       = received_frame[7];
  address_r[4]       = received_frame[8];
  address_r[5]       = received_frame[9];
  address_r[6]       = received_frame[10];
  address_r[7]       = received_frame[11];
  
  //rssi.
  rssi_r             = received_frame[12];
      
  //options.
  options_r          = received_frame[13];
      
  //data.
  for(int i1= 0; i1 < received_frame_length - 11; i1++)
  {
    rf_data_r[i1] = received_frame[14+i1];
  }
       
  //checksum.
  checksum_r         = received_frame[3 + received_frame_length];
  
  //verificación de la integridad de los datos recibidos.
  if(checksum_r == getChecksum(received_frame, 3 + received_frame_length))
  {
    //impresión.
    Serial.println("");
    Serial.println("Datos recibidos exitosamente.");
  }
  
  else
  {
    //impresión.
    Serial.println("");
    Serial.println("Datos no recibidos exitosamente.");
  }
  
  //impresión de datos recibidos.
  printReceivedData();
}

//------------------------------------------------
//función para guardar datos de los sensores en SD.
void saveDataInSD(int *_data_in, int _samples_amount)
{
  //variable auxiliar.
  String dataString = "";
  
  //se abre o crea un fichero.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  //si es archivo está disponible, entonces.
  if (dataFile) 
  {
    //bucle crear variable string con datos sensados.
    for(int i1= 0; i1 < _samples_amount; i1++)
    {
      //seteo de los datos sensados.
      dataString += String(_data_in[i1]);

      //impresión de espacio.
      if (i1 < _samples_amount - 1) dataString += ", "; 
    }

    //escritura en el fichero.
    dataFile.println(dataString);

    //cierre del fichero.
    dataFile.close();
  }  
  
  //si el fichero no está disponible, entonces.
  else 
  {
    //inpresión de error.
    Serial.println("error opening datalog.txt");
  } 
}

//------------------------------------------------
//función para imprimir recepción de datos.
void printReceivedData()
{
  //impresión.
  Serial.print("Frame: ");
  
  //bucle para imprimir datos.
  for(int i1= 0; i1 < received_frame_length + 4; i1++)
  {
    //impresión de una frame.
    Serial.print(received_frame[i1], HEX); 
    
    //impresión de espacio.
    Serial.print(" ");
  }

  //impresión de dos nuevsa líneas.
  Serial.println("");
  Serial.println("");  

  //impresión de start_delimiter_r.
  Serial.print("start delimiter  : ");
  Serial.print(start_delimiter_r, HEX);
  Serial.println(""); 

  //impresión de frame_length_r.
  Serial.print("frame length     : ");
  Serial.print(frame_length_r[0], HEX);
  Serial.print(" ");
  Serial.print(frame_length_r[1], HEX);
  Serial.println(""); 

  //impresión de frame_type_r.
  Serial.print("frame type       : ");
  Serial.print(frame_type_r, HEX);
  Serial.println(""); 

  //impresión de address_r.
  Serial.print("origin address   : ");
  Serial.print(address_r[0], HEX);
  Serial.print(" ");
  Serial.print(address_r[1], HEX);
  Serial.print(" ");
  Serial.print(address_r[2], HEX);
  Serial.print(" ");
  Serial.print(address_r[3], HEX);
  Serial.print(" ");
  Serial.print(address_r[4], HEX);
  Serial.print(" ");
  Serial.print(address_r[5], HEX);
  Serial.print(" ");
  Serial.print(address_r[6], HEX);
  Serial.print(" ");
  Serial.print(address_r[7], HEX);
  Serial.println(""); 

  //impresión de rssi_r.
  Serial.print("rssi             : ");
  Serial.print(rssi_r, HEX);
  Serial.println(""); 

  //impresión de options_r.
  Serial.print("options          : ");
  Serial.print(options_r, HEX);
  Serial.println(""); 

  //impresión de rf_data_r.
  Serial.print("rf_data          : ");

  //bucle para imprimir datos.
  for(int i1= 0; i1 < received_frame_length - 11; i1++)
  {
    //impresión de datos.
    Serial.print(rf_data_r[i1], HEX);
    Serial.print(" ");
  }

  //impresión de nueva línea.
  Serial.println(""); 

  //impresión de checksum_r.
  Serial.print("checksum         : ");
  Serial.print(checksum_r, HEX);
  Serial.println(""); 

  //impresión de una nueva linea.
  Serial.println("");  
}

//------------------------------------------------
//función para imprimir mediciones de datos.
void printDataOfSensors(int *_data_in, int _samples_amount)
{
  //bucle para imprimir datos.
  for(int i1= 0; i1 < _samples_amount; i1++)
  {
    //impresión de mediciones.
    Serial.print(_data_in[i1]);
  
    //impresión de espacio.
    if (i1 < _samples_amount - 1) Serial.print(", ");
  }

  //impresión de espacio.
  Serial.println("");
}

//------------------------------------------------
//función para obtener checksum.
byte getChecksum(byte _frame[], int _frame_length)
{
  //variable auxiliar.
  byte sumChecksum = 0;
 
  //bucle para calcular suma de Checksum.
  for(int i1= 3; i1 < (_frame_length - 1); i1++)
  {
    sumChecksum += _frame[i1];
  }
 
  //checksum.
  sumChecksum   = 0xFF - sumChecksum;

  //retorno del checksum calculado.
  return sumChecksum;
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//funciones limpieza de buffers.

//-------------------------------------------------------------------------------------
//función para limpiar buffers de entrada de datos, salida de datos y buffer stream.
void cleanIOSBuffers(void)
{
  //limpieza de buffer auxiliares.
  stream_index_available  = 0;
  stream_available_bits   = 8;

  //bucle para limpiar "stream_com_data".
  for (int i1 = 0; i1 < MAX_BUFFER_SIZE; i1++)
  {
    //limpieza.
    stream_com_data[i1] = 0;
  }

  //bucle para limpiar buffers.
  for (int i1 = 0; i1 < samples_amount; i1++)
  {
    //limpieza.
    data_in[i1]   = 0;
    data_out[i1]  = 0;
  }
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//funciones lectura de datos e inicialización.

//------------------------------------------------
//función para inicializar buffer de envío y recepción.
void initializedBufferToSendAndReceptionData()
{
	//----------------------------------------------
  //inicialización de estructura de frame recibido.
  start_delimiter_r = 0x7E;

  //----------------------------------------------
  //inicialización de bufer de frame recibido.
  received_frame[0] = 0x7E;

  //----------------------------------------------
  //start delimiter.
  send_frame[0]     = 0x7E;

  //----------------------------------------------
  //length.
  send_frame[1]     = 0x00;
  send_frame[2]     = 0x1F;

  //----------------------------------------------
  //frame type.
  send_frame[3]     = 0x00;

  //----------------------------------------------
  //64 bit address of destination.
  send_frame[5]     = 0x00;
  send_frame[6]     = 0x13;
  send_frame[7]     = 0xA2;
  send_frame[8]     = 0x00;
  send_frame[9]     = 0x40;
  send_frame[10]    = 0x71;
  send_frame[11]    = 0x62;
  send_frame[12]    = 0x73;

  //----------------------------------------------
  //options.
  send_frame[13]    = 0x00;
}

//------------------------------------------------
//función para inicializar "dataloger".
void initializedDataloger()
{
	//se abre o crea un fichero.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  //si es archivo está disponible, entonces.
  if(dataFile)
  {
    //impresión de nueva línea.
    dataFile.println("");
    
    //se salva la fecha.
    /*dataFile.print("fecha: ");
    dataFile.print(day(datatime));
    dataFile.print("/");
    dataFile.print(month(datatime));
    dataFile.print("/");
    dataFile.print(year(datatime));
    dataFile.print("    ");
    
    //se salva la hora.
    dataFile.print("hora: ");
    dataFile.print(hour(datatime));
    dataFile.print(":");
    dataFile.print(minute(datatime));
    dataFile.print(":");
    dataFile.print(second(datatime));
    dataFile.println("");*/

    //se salva la leyenda de los datos.
    dataFile.print("tipos de datos: ");
    dataFile.print("Temperatura.");
    dataFile.println("");
    dataFile.println("");
   
    //cierre del fichero.
    dataFile.close();
  }
}
	

//-------------------------------------------------------------------------------------
//función para leer buffer de set de datos de prueba.
void redBufferDataSet(int *_data_in, int _samples_amount, int *_dataset, int &_dataset_index)
{
  //bucle para leer buffer de set de datos.
  for (int i1 = 0; i1 < _samples_amount; i1++)
  {
    //seteo de set de datos desde buffer.
    _data_in[i1] = _dataset[_dataset_index + i1];   
  }

  //actualización de "_dataset_index".
  _dataset_index += _samples_amount;
}

//-------------------------------------------------------------------------------------
//función para leer fichero de set de datos de prueba. 
void readFileDataSet(char *_filename, int *_dataset, int _length_dataset)
{
  //valor leído.
  char  val_c[6];

  //se abre el fichero con el set de datos.
  File dataFile = SD.open(_filename);
  
  //bucle para leer fichero. 
  for (int i1 = 0; i1 < _length_dataset; i1++)
  {
    //lectura del fichero.
    dataFile.read(val_c, 6); 

    //seteo del valor leído.
    _dataset[i1] = (val_c[0] - 48)*1000 +  (val_c[1] - 48)*100 +  (val_c[2] - 48)*10 + (val_c[3] - 48);
  }

  //cierre del fichero.
  dataFile.close();
}


#endif

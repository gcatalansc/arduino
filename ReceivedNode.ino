
//###########################################################################
/*
  ReceivedNode: Las distintas funciones que tiene este código son:
  
              # Código para recibir datos comprimidos desde sensor inalámbrico.
              # Enviar paquete de datos comprimidos por puerto serial a Gateway.
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
#define BUFFER_RECEIVED_BUFFER_SIZE 2048

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//variables globales.

//pines para LoRa;
const int csPin                       = 10;                                   //selección de radio chip. 
const int resetPin                    = 9;                                    //radio reset.
const int irqPin                      = 2;                                    //pin de interrupción.

//dirección local.
byte      localAddress                = 0xB1;

//cabecera de paquete recibido.
int       destination_address         = 0;                                    //dirección de destino.
byte      source_address              = 0;                                    //dirección de fuente.
byte      data_id                     = 0;                                    //id de paquete recibido.
byte      data_length                 = 0;                                    //largo del payload recibido.

//índice de byte leido.
int       read_data_index             = 0;

//buffers de lectura de datos.
byte      received_buffer[BUFFER_RECEIVED_BUFFER_SIZE];

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//setup.
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
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//loop principal.
void loop() 
{
  //función para recibir paquetes de datos.
  onReceive(LoRa.parsePacket()); 
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//función para recibir paquetes de datos.
void onReceive(int packetSize) 
{
  //si no se han recibido paquetes, entonces retornar.
  if (packetSize == 0) return;          

  //lectura de cabecera del paquete recibido.
  destination_address   = (byte)LoRa.read(); 
  source_address        = (byte)LoRa.read(); 
  data_id               = (byte)LoRa.read(); 
  data_length           = (byte)LoRa.read(); 

  //llenado de buffer de recepción de datos.
  received_buffer[0]    = destination_address;
  received_buffer[1]    = source_address;
  received_buffer[2]    = data_id;
  received_buffer[3]    = data_length;
  received_buffer[4]    = (byte)LoRa.packetRssi();
  received_buffer[5]    = (byte)LoRa.packetSnr();

  //reset del índice de bytes leídos.
  read_data_index       = 5;

  //bucle de lectura de paquete recibido
  while (LoRa.available()) received_buffer[++read_data_index] = (byte)LoRa.read();
  
  //for(int i1 = 5; i1 < (read_data_index + 1); ++i1) Serial.println(received_buffer[i1]);      
  
  //si el largo del paquete recibido es inconsistente, entonces.
  if (data_length != (read_data_index - 5)) return;

  //si la dirección de destino del paquete recibido es inconsistente, entonces.
  if (destination_address != 0xFF)          return;   

  //envío de paquete de datos a Gateway.
  //Serial.write(received_buffer, (read_data_index + 1));
}






#include <SPI.h>              // include libraries
#include <LoRa.h>

//Funciones
int bitCount(int _code);
void printMeasure(String prefix, float value, String postfix);
int getDataLossAmount(int data_id);
int counter = 0;
int counterE = 0;
int tem[100], dato[100],temE[100],datoE[100];
int i = 1,z=1,p=0;
int u,w, code;
float suma , tase=300, sumabit,sumaE , taseE=1, sumabitE;

const int analogPin = A0;
int value;      //variable que almacena la lectura analógica raw
int bite=0,bitE=0;
unsigned long tiempo;
unsigned long time_ms;
unsigned long tiempoE;
unsigned long time_msE, time_r= 0, time_rx = 0, time_e= 0, time_ex = 0;
//unsigned long zet;
const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xB1;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time

//variables para obtener tasa de pérdida de datos.
int data_loss_counter = 0;
int data_id_r         = 0;


//buffer para recepción de datos.
byte received_buffer[100];
bool try_camera_state_flag  = true;
bool data_finished_flag     = false;

void setup() {
  Serial.begin(9600);                   // initialize serial
  Serial1.begin(9600);                   // initialize serial
  
  while (!Serial);
  
  //Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  //bucle para inicializar LoRa.
  while (!LoRa.begin(915E6));
  
  LoRa.setTxPower(14,PA_OUTPUT_RFO_PIN);
  LoRa.setFrequency(866E6);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  //Serial.println("LoRa init succeeded. TEENSY");

  time_rx =  millis();
  time_ex = time_rx;
}
///%%%%%%%%%%%%LOOP PRINCIPAL%%%%%%%%%%
///####################################

void loop() 
{
  //variables auxiliares.
  byte index             = 0;
  byte read_buffer       = 0;
  long set_time          = 0;
  long startCamDataTime  = 0;
  long stopCamDataTime   = 0;  
  int  data_available    = 0;
  int  bytes_available   = 0;
  try_camera_state_flag  = true;
  data_finished_flag     = false;
  
  //si se ha consultado el estado de al sincronización, entonces.
  if(Serial.available() == 1)
  {
    //captura del dato recibido.
    read_buffer = Serial.read();
    
    //verificación del comando, entonces.
    if(read_buffer == 's')
    {          
      //bucle para obtener el estado de la cámara.
      while(try_camera_state_flag)
      {
        //seteo de la notificación.
        outgoing = "s";
        
        //enviar aviso del término del envío de la imagen.
        sendMessage(outgoing);

        //captura de tiempo de detensión.
        stopCamDataTime = millis();
       
        //espera hasta recibir respuesta del estado de la cámara.
        while(((data_available = LoRa.parsePacket()) == 0) && 
              ((millis() - stopCamDataTime) < 5000));
         
        //recepción de la respuesta.
        if(data_available == 5)  
        {
          //recepción de la respuesta.
          onReceiveM();         
        }
      } 
    }   
  }
  
  //si se han recibido datos de configuración, entonces.
  else if(Serial.available() == 7)
  {   
    //captura del dato recibido.
    read_buffer = Serial.read();
   
    //seteo de buffer de salida.
    outgoing = read_buffer;

    //incremento del índice.
    ++index;
      
    //bucle para obtener los demás parámetros.
    while(index < 7)
    {
      //si hay datos disponibles.
      if(Serial.available() > 0)
      {
        //captura del dato recibido.
        read_buffer = Serial.read();
          
        //seteo de buffer de salida.
        outgoing += read_buffer;

        //incremento del índice.
        ++index;
      }        
    }

    //envío de parámetros de configuración.
    sendMessage(outgoing);

    //bucle para obtener todos los paquetes enviados.
    while(!data_finished_flag)
    {
      //captura de tiempo de detensión.
      stopCamDataTime = millis();
        
      //espera hasta recibir datos de la imagen.
      while(LoRa.parsePacket() == 0)
      {
        //conteo del tiempo.
        if((millis()- stopCamDataTime) > 3000)    
        {         
          //seteo de bandera.
          data_finished_flag = true;

          //envío de respuesta.
          Serial.write('f');
          
          //salida del bucle.
          break;
        }
      }

      //recepción de datos de la imagen tomada.
      if(!data_finished_flag) onReceiveI(); 
    }  
  }  
}

//######################################################################
//############             RECEPCION            ########################
//######################################################################

//función para recibir paquetes de respuestas.
void onReceiveM() 
{    
  //read packet header bytes:
  int recipient       = LoRa.read();      // recipient address   try_camera_state_flag
  int sender          = LoRa.read();      // sender address
  int incomingMsgId   = LoRa.read();      // incoming msg ID
  int incomingLength  = LoRa.read();      // incoming msg length
  int index           = 0;

  //bucle para leer respuesta.
  while (LoRa.available()) 
  {
    //lectura de respuesta.
    received_buffer[index] =  (char)LoRa.read();

    //incremento del índice.
    ++index;
  }

  if(received_buffer[0] != 'f') 
  {
    //envío de respuesta.
    Serial.write(received_buffer, index);

    //seteo de bandera.
    try_camera_state_flag = false;
  }
}

//función para recibir paquetes de la imagen.
void onReceiveI() 
{  
  //read packet header bytes:
  int recipient       = LoRa.read();      // recipient address
  int sender          = LoRa.read();      // sender address
  int incomingMsgId   = LoRa.read();      // incoming msg ID
  int incomingLength  = LoRa.read();      // incoming msg length
  int index           = 0;

  //bucle para leer respuesta.
  while (LoRa.available()) 
  {
    //lectura de respuesta.
    received_buffer[index] =  (char)LoRa.read();

    //incremento del índice.
    ++index;
  } 

  //si no es el fin de la iamgen, entonces entonces enviar pixeles..
  if(index > 1)   Serial.write(received_buffer, index); 

  //si es el fin de la imagen, entonces setear bandera.   
  else            data_finished_flag = true;  
}

//############################################################
//###################   ENVIO DE DATOS        ################
//############################################################
void sendMessage(String _outgoing) 
{
  //time_ms = millis();
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(_outgoing.length());        // add payload length
  LoRa.print(_outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  counter++;
  i++;
}

  
///#########################################################################
///#########              FUNCIONES GENERALES          #######################
///#########################################################################

//$$$$$$$$$$$$$$$$DAR FORMATO $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

void printMeasure(String prefix, float value, String postfix)
{
   Serial.print(prefix);
   Serial.print(value, 3);
   Serial.println(postfix);
}



//#############FUNCION PARA CALCULAR BIT################################################//
  
int bitCount(int _code)
{
int int_bytes_amount = sizeof(_code);

 //si el valor es cero, entonces.
  if (_code == 0)     return 0;

  //---------------------------------------------------------------------------------
  //bucle para obtener cantidad bits utilizados.
  for (int i1 = (int_bytes_amount * 8 - 1); i1 >= 0; i1--)
  {
    //si existe un bit en 1, entonces.
    if ((_code & (1 << i1)) != 0) return (i1 + 1);
  }

  return 0;

}

//#######################################################################


//#######################################################################
//##########Función para obtener la cantidad de datos perdidos.##########
int getDataLossAmount(int data_id)
{
  return (data_id - data_id_r - 1);
}




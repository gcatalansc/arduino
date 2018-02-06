
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "uCam.h"

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

byte    received_buffer[100];
int     payload = 64;

//declaración de la cámara.
UCAM529 camera(&Serial1);

void setup() {

  //seteo de baudios.
  Serial.begin(115200);
  Serial1.begin(115200);

  //impresión.
  Serial.println("> Wellcome...");
 
  //espera.
  delay(2000);
 
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  //bucle para inicializar LoRa.
  while (!LoRa.begin(915E6));
  
  LoRa.setTxPower(14,PA_OUTPUT_RFO_PIN);
  LoRa.setFrequency(866E6);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(250E3);

  //impresión.
  Serial.println("> LoRa inicializada");

  //inicialización de bandera.
  camera.cam_sync = false;

  //impresión.
  Serial.print("> sincronizando camara...");
  
  //bucle para sincronizar cámara.
  while (!camera.cam_sync)  camera.init();

  //enviando ack.
  camera.send_ack();

  //impresión.
  Serial.println(" OK");
}
///%%%%%%%%%%%%LOOP PRINCIPAL%%%%%%%%%%
///####################################

void loop() 
{   
  //impresión.
  Serial.println("");
  Serial.println("> espera por consulta o comando.");
  
  //espera hasta recibir datos de la imagen.
  while(!LoRa.parsePacket());

  //impresión.
  Serial.println("> lectura de comando");
  
  //recepción comando.
  onReceiveM();  
}

//######################################################################
//############             RECEPCION            ########################
//######################################################################

void onReceiveM()
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

  //si se ha consultado por el estado de la sincronización, entonces.
  if(index == 1 && received_buffer[0] == 's')
  {
    //seteo de la notificación.
    outgoing = "s";
        
    //enviar aviso del término del envío de la imagen.
    sendMessage(outgoing);    
  }

  //si se recibieron los 7 parámetros, entonces.
  else if(index == 7)
  {    
    camera._INITIAL[3]     = received_buffer[0];
    camera._INITIAL[4]     = received_buffer[1] - 48;
    camera._SNAPSHOT[2]    = received_buffer[5] - 48;
    camera._GET_PICTURE[2] = received_buffer[6] - 48;
    
    //impresión.
    Serial.print("> send_initial...");

    //seteo de parámetros iniciales.
    if (camera.send_initial()) 
    {
      //impresión.
      Serial.println(" OK");
      Serial.print("> do_snapshot...");
    
      //seteo de parámetros para snapshot.
      if (camera.do_snapshot()) 
      {
        //impresión.
        Serial.println(" OK");
        Serial.print("> get_picture...");
      
        //seteo de parámetros para el tipo de picture.
        if (camera.get_picture()) 
        {
          //impresión.
          Serial.println(" OK");
      
          //petición de datos.
          camera.get_data(); 

          //impresión.
          Serial.println("> configuracion finalizada");          
      
          //captura de datos.
          get_raw_picture_data();

          //impresión.
          Serial.println("");
          Serial.print("> sincronizando camara...");
          
          //bucle para sincronizar cámara.
          while (!camera.cam_sync)  camera.init();
        
          //enviando ack.
          camera.send_ack();
        
          //impresión.
          Serial.println(" OK");
        }
      }
    }          
  }
}

//############################################################
//###################   ENVIO DE DATOS        ################
//############################################################
void sendMessage(String _outgoing) 
{
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

void sendImage(String _outgoing) 
{
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(_outgoing.length());                       // add payload length
  LoRa.print(_outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  counter++;
  i++;
}

///#########################################################################
///#########              FUNCIONES PARA IMAGEN        #####################
///#########################################################################

void get_raw_picture_data() 
{
  //variables auxiliares.
  String  image[640];
  int     totalBytes       = 0;
  int     index            = 0;
  int     bytes_for_index  = 0;
  long    startCamDataTime = 0;
  long    stopCamDataTime  = 0;

  //impresión.
  Serial.print("> capturando datos de la imagen...");

  //captura de tiempo inicial.
  startCamDataTime  = millis();  
  
  //lectura de datos.
  while (Serial1.available()) 
  {
    //lectura de datos.
    image[index] += (char)Serial1.read();
    
    //incremento del contador de bytes por fila.
    ++bytes_for_index;

    //cantidad de bytes para enviar datos.
    if (bytes_for_index == payload) 
    {     
      //incremento del índice.
      ++index;

      //reset del contador de bytes por fila.
      bytes_for_index = 0;
    }
          
    //captura de tiempo de detensión.
    stopCamDataTime = millis();

    //espera.
    while (!Serial1.available() && millis()-stopCamDataTime < 100); 
  }

  //enviando ack.
  camera.send_ack_picture();

  //impresión.
  Serial.println(" OK");
  Serial.print("> enviando datos...");

  //bucle para enviar datos de la imagen.
  for(int i1 = 0; i1 < (index + 1); ++ i1)
  {
    //si al imagen posee pixeles, entonces.
    if(image[i1].length() > 0)
    {
      //enviar comando para sincronizar cámara.
      sendImage(image[i1]); 

      //suma de bytes enviados
      totalBytes += payload;
    
      /*//bucle para enviar datos de la imagen.
      for(int i2 = 0; i2 < image[i1].length(); ++ i2)
      {
        //impresión de imagen.
        Serial.print(image[i1][i2],DEC);
        Serial.print(" ");
      }  

      //impresión de imagen.
      Serial.println();*/
    
      //retraso.
      //delay(5);
    }    
  }

  //mensaje de aviso de término de envío de datos.
  outgoing = "f";
          
  //enviar aviso de término de envío de datos.
  sendMessage(outgoing);

  //impresión.
  Serial.println(" OK");
  Serial.print("> total de bytes enviados: ");
  Serial.println(totalBytes);
  
}
  
///#########################################################################
///#########              FUNCIONES GENERALES        #######################
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




//Control Arduino UNO with BLE HM-10C/C++
//////////////////////////////////////////////////
//
// Small BLE app demo
// (c) 2019, Nicolin Cioroianu
//  Using:
//     Upload the sketch on your Arduino
//     Connect the HM-10 to your
//     
//     Via BLE App seriel App customize zour Arduino actions
//     example: turn ON/OFF a led
/////////////////////////////////////////////////////



///////////////////////////////////////////
//
// 1. OSINO SCHEDULER CONFIGURATION
//
///////////////////////////////////////////
#define TASK_5MS    OSINO_SCHED1
#define PERIOD_OSINO_SCHED1 5u
#define TASK_50MS   OSINO_SCHED2
#define PERIOD_OSINO_SCHED2 50u
#define TASK_100MS  OSINO_SCHED3
#define PERIOD_OSINO_SCHED2 100u
//#define TASK_xxx  OSINO_SCHED4
//...add here tasks



int osino_task_activation[OSINO_NUMBER_OF_TASKS]

// we aere using Arduino Software Serial apis
#define AduinoSoftSerialLIB

// config LEDs
//const int LED  = 13;
//const int LED_ext  = A0;
typedef enum{
  E_UNDEF=0,
  E_INIT,
  E_OK,
  E_PENDING,
  E_NOK
}T_E_COM_STATE;

T_E_COM_STATE re_ComState= E_UNDEF;

#ifdef AduinoSoftSerialLIB
  #include <SoftwareSerial.h>
  //const int BTRX = 2;  // 11
  const int BTRX = 0;
  //const int BTTX = 3;  // 10
  const int BTTX = 1;
  //const int BTRTS = 6;
  const int BTRTS = 5;
  const int BTCTS = 2;  // 10
  SoftwareSerial SerialBT(BTRX, BTTX);
#else 
  HardwareSerial SerialBT = Serial1;
#endif


// Die versendete Nachricht:
String msg; 



void ble_IO_init(void){
  SerialBT.begin(9600);
  SerialBT.println("Communication Estabilished");
  //pinMode(LED, OUTPUT);
  pinMode(BTRTS, OUTPUT);

  //pinMode(BTCTS, INPUT);
  pinMode(BTCTS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTCTS), onCTS_Notification, CHANGE);
}



void arduino_tsr_init(void){
  TCCR0A=(1<<WGM01);    //Set the CTC mode   
  OCR0A=0xF9; //Value for ORC0A for 1ms 
  TIMSK0|=(1<<OCIE0A);   //Set the interrupt request
  sei(); //Enable interrupt
  TCCR0B|=(1<<CS01);    //Set the prescale 1/64 clock
  TCCR0B|=(1<<CS00);
}
///////////////////////////////////////////////////
//
// setup
//    communication with HM-10 
//
///////////////////////////////////////////////////

void setup() {
  ble_IO_init();
  arduino_tsr_init();
  task_add(TASK_5MS);
  task_add(TASK_50MS);
  task_add(TASK_100MS);

}

///////////////////////////////////////////////////
//
// loop
//    In each itteration check the Messsage
//
///////////////////////////////////////////////////

void loop() {
  int lu8_isAvailable = SerialBT.available();
  int lu8_isCTS = digitalRead(BTCTS);
  if ((lu8_isAvailable)&&(lu8_isCTS)){      // checf if there is data
     msg = SerialBT.readString(); // get the message
     switch(msg):

	 SerialBT.println("Executed" + CMD1);
      }
      else {
         SerialBT.print("Command <");
         SerialBT.print(msg);
         SerialBT.println("unknown");
      }
    }
}
/*=============================================
 * Interrupts
===============================================*/
ISR(TIMER0_COMPA_vect){    //This is the interrupt request
  //timer++;
}
void onCTS_Notification(void){
  //have to check here
}

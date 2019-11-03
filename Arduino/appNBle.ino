//Control Arduino UNO with BLE HM-10C/C++
//////////////////////////////////////////////////
//
// Small BLE app demo
// (c) 2019, Nicolin Cioroianu
//  Using:
//     Upload the sketch o your Arduino
//     An Arduino HM-10 anschließen
//     
//     Über BLE App serielle Verbindung aufbauen,
//     "ein" oder "aus" ins Textfeld eingeben, 
//     und LED beobachten.
//
///////////////////////////////////////////////////



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

// Wir verwenden Software Serial
#define AduinoSoftSerialLIB

// Eingebaute LED nutzen:control
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
  SerialBT.println("Bluetooth-Verbindung steht");
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
//    Verbindung mit HM-10 aufbauen
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
//    In jeder Iteration auf Nachricht warten,
//    Nachricht analysieren,
//    Aktion auslösen (LED ein/aus)
//
///////////////////////////////////////////////////

void loop() {
  int lu8_isAvailable = SerialBT.available();
  int lu8_isCTS = digitalRead(BTCTS);
  if ((lu8_isAvailable)&&(lu8_isCTS)){      // Daten liegen an
     msg = SerialBT.readString(); // Nachricht lesen
     if (msg == "ein") {
//         digitalWrite(LED, HIGH);
//         digitalWrite(LED_ext, HIGH);
//         SerialBT.print("LED an Pin ");
//         SerialBT.print(LED);
//         SerialBT.println(" ist eingeschaltet!");
      } 
      else
      if (msg == "aus") {
//         digitalWrite(LED, LOW);
//         digitalWrite(LED_ext, LOW);
//         SerialBT.print("LED an Pin ");
//         SerialBT.print(LED);
//         SerialBT.println(" ist ausgeschaltet!");
      }
      else {
         SerialBT.print("Kommando <");
         SerialBT.print(msg);
         SerialBT.println("> nicht bekannt");
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

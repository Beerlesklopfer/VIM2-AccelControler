
#define IS_WITHOUT_HARDWARE  
#define DATA_FORMAT          1

#define DRV8825_DIR_PIN     13
#define DRV8825_ENA_PIN      8
#define DRV8825_STEP_PIN     6
#define DRV8825_FLT_PIN      2
#define DRV8825_ENA_PIN      3
#define DRV8825_SLP_PIN     12

#define DRV8825_M0_PIN       9
#define DRV8825_M1_PIN      10
#define DRV8825_M2_PIN      11

#define ENCODER_A_PIN       18
#define ENCODER_B_PIN       19

#define DEBUGGING_PORT Serial    // Serial port to use for RS485 communication, change to the port you're using.
#define STREAMING_PORT Serial2   // Serial port to use for RS485 communication, change to the port you're using.
#define MODBUS_PORT    Serial3   // Serial port to use for RS485 communication, change to the port you're using.

#define SLAVE_ID                1                  // The Modbus slave ID

#define MODBUS_BAUDRATE      9600                  // modbus baudrate
#define STREAMING_BAUDRATE 500000                  // streaming baudrate
#define DEBUGGING_BAUDRATE 115200                  // debugging baudrate

#define MB_COIL_STEPPER_DIR_CCW             1      // stepper direction 0=CW; 1=CCW 
#define MB_COIL_STEPPER_ENA                 2      // Enables the stepper
#define MB_COIL_STEPPER_RUN                 3      // Run the stepper 
#define MB_COIL_STEPPER_MOVE                4      // Run the stepper clockwise
#define MB_COIL_START_MEASSURE              5      // Start one measurement cycle
#define MB_COIL_ZERO_ENCODER                6      // sets the encoder to zero        

#define MB_CONTACT_STEPPER_IS_RUNNING   10001      // indicates weather the stepper is running
#define MB_CONTACT_STEPPER_IS_STOPPING  10002      // indicates weather the stepper is stopping

#define MB_INPUT_SYSTEM_TIME_LSB        30001      // current system Time (LSB)
#define MB_INPUT_SYSTEM_TIME_MSB        30002      // current system Time (MSB)
#define MB_INPUT_SYSTEM_TIME_START_LSB  30003      // current system Time at start (LSB)
#define MB_INPUT_SYSTEM_TIME_START_MSB  30004      // current system Time at start (MSB)
#define MB_INPUT_SYSTEM_TIME_FINISH_LSB 30005      // current system Time at finish (LSB)
#define MB_INPUT_SYSTEM_TIME_FINISH_MSB 30006      // current system Time at finish (MSB)
#define MB_INPUT_STEPS_STEPPER          30007      // Current steps from the stepper driver
#define MB_INPUT_STEPS_ENCODER          30009      // current steps from the encoder
#define MB_INPUT_ANGLE_ENCODER          30010      // current (calculated) Angle from the encoder [rad*10000]
   
#define MB_HOLDING_STEPPER_TURN_STEPS   40001      // stepper: amout of steps per turn
#define MB_HOLDING_ENCODER_TURN_STEPS   40002      // encoder: amout of steps per turn
#define MB_HOLDING_MESSURE_TIME         40003      // length of measurement [ms]
#define MB_HOLDING_SAMPLE_RATE          40004      // samplingrate of measurement [Hz]
#define MB_HOLDING_STEPER_MICROSTEPS    40005      // Micosteps from 2^0..2^4
#define MB_HOLDING_STEPPER_ACCEL        40006      // turning speed [ω/s]
#define MB_HOLDING_STEPPER_SPEED        40007      // turning acceleration [ω/s²]
#define MB_HOLDING_STEPPER_TURNS        40008      // amount ofsteps to run 0=inf

#include <Arduino.h>
#include <Encoder.h>
#include <HardwareSerial.h>
#include <ModbusSlave.h>
#include <FastAccelStepper.h>
#include <EEPROM.h>
#include <TimerThree.h>

#ifdef IS_WITHOUT_HARDWARE
  #include <TimerFour.h>
#endif

// SoftwareSerial modBusSerial(MB_RX_PIN, MB_TX_PIN);
Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);
Modbus mbSlave(MODBUS_PORT, (uint8_t)SLAVE_ID);
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper *stepper;

#ifdef IS_WITHOUT_HARDWARE
  double gRadTimeStamp=0;    // Sampledate for sine
#endif

enum MICROSTEPPING {FULL_STEP=0, HALF_STEP=1, QUARTER_STEP=2, 
                    OCTA_STEP=3, HEXA_STEP=4, MAX_STEP=7};

// coils
volatile static bool gCoils[30]={
  true, // [00001] Stepper direction (false=CW, true/CCW)
  true,  // [00002] Stepper Outputs were enabled
  false, // [00003] Stepper has controlled my run command
  false, // [00004] Stepper has controlled my move command
  false, // [00005] A meassure is running
  false, // [00006] The encoder has been zeroed
  false, // [00007] 
  false, // [00008] 
  false, // [00009] 
  false, // [00010] 
  false, // [00011] 
  false, // [00012] 
  false, // [00013] 
  false, // [00014] 
  false, // [00015] 
  false, // [00016] 
  false, // [00017] 
  false, // [00018] 
  false, // [00019] 
  false, // [00020] 
  false, // [00021] 
  false, // [00022] 
  false, // [00023] 
  false, // [00024] 
  false, // [00025] 
  false, // [00026] 
  false, // [00027] 
  false, // [00028] 
  false, // [00029] 
  false  // [00030] 
};

#define gStepperDirection      gCoils[0]  // Stepper direction (false=CW, true/CCW)
#define gStepperEnableOutputs  gCoils[1]  // Stepper Outputs were enabled
#define gStepperRun            gCoils[2]  // Stepper has controlled by run command
#define gStepperMove           gCoils[3]  // Stepper has controlled by move command
#define gStartMeassure         gCoils[4]  // A meassure is running
#define gSetEncoderZero        gCoils[5]  // The encoder has been zeroed

// discrete inputs
volatile static bool gContacts[30]={
  false, // [10001] stepper is running
  true,  // [10002] stepper is stopping
  false, // [10003] messure is running
  false, // [10004] 
  false, // [10005] 
  false, // [10006] 
  false, // [10007] 
  false, // [10008] 
  false, // [10009] 
  false, // [10010] 
  false, // [10011] 
  false, // [10012] 
  false, // [10013] 
  false, // [10014] 
  false, // [10015] 
  false, // [10016] 
  false, // [10017] 
  false, // [10018] 
  false, // [10019] 
  false, // [10020] 
  false, // [10021] 
  false, // [10022] 
  false, // [10023] 
  false, // [10024] 
  false, // [10025] 
  false, // [10026] 
  false, // [10027] 
  false, // [10028] 
  false, // [10029] 
  false  // [10030] 
};
#define gStepperIsRunning      gContacts[0]  // Stepper has controlled by run command
#define gStepperIsStopping     gContacts[1]  // Stepper has controlled by move command
#define gMessureIsRunning      gContacts[2]  // messure is running

// analog input
volatile static uint16_t gInputs[30]={
  0,    // [30001] system Time 1/4
  0,    // [30002] system Time 2/4
  0,    // [30003] system Time 3/4
  0,    // [30004] system Time 4/4
  0,    // [30005] system Time at start 1/4
  0,    // [30006] system Time at start 2/4
  0,    // [30007] system Time at start 3/4
  0,    // [30008] system Time at start 4/4
  0,    // [30009] system Time at finish 1/4
  0,    // [30010] system Time at finish 2/4
  0,    // [30011] system Time at finish 3/4
  0,    // [30012] system Time at finish 4/4
  0,    // [30013] current Encoder position 1/4
  0,    // [30014] current Encoder position 2/4
  0,    // [30015] current Encoder position 3/4
  0,    // [30016] current Encoder position 4/4
  0,    // [30017] current Encoder value 1/4
  0,    // [30018] current Encoder value 2/4
  0,    // [30019] current Encoder value 3/4
  0,    // [30020] current Encoder value 4/4
  0,    // [30021] Value of current Stepper value 1/4
  0,    // [30022] Value of current Stepper value 2/4
  0,    // [30023] Value of current Stepper value 3/4
  0,    // [30024] Value of current Stepper value 4/4
  0,    // [30025]
  0,    // [30026]
  0,    // [30027]
  0,    // [30028]
  0,    // [30029]
  0     // [30030] 
};       
volatile unsigned long gSystemTime           = millis();  // system Time since powering up MCU
volatile unsigned long gStartTime            = millis();  // system Time at start
volatile unsigned long gFinishTime           = millis();  // system Time at finish
volatile unsigned long gCurrentEncoderPostition = 0L;        // current Encoder position
volatile unsigned long gCurrentEncoderValue  = 0L;        // current Encoder value
volatile unsigned long gLastEncoderAngle     = 0L;        // last EncoderPosition
volatile unsigned long gCurrentStepperPosition = 0L;        // Value of current Stepper position

// analog IO
volatile  uint16_t gHolding[30]={
  200,     // [40001] amout of steps per turn (stepper settings)
  400,     // [40002] amout of steps per turn (encoder settings)
  2000,    // [40003] duration of messurement [ms]
  1,       // [40004] samplingrate of measurement [Hz]
  0,       // [40005] turning speed [ω/s]
  0,       // [40006] turning acceleration [ω/s²]
  0,       // [40007] microsteps (multiplier for gHolding[0])
  0,       // [40008] amount of steps to run 1/4
  0,       // [40009] amount of steps to run 2/4
  0,       // [40010] amount of steps to run 3/4
  0,       // [40011] amount of steps to run 4/4
  0,       // [40012]  
  0,       // [40013]  
  0,       // [40014]  
  0        // [40015] 
};       
#define gStepperSteperPerTurn     gHolding[0]   // Stepper has controlled by move command
#define gEncoderSteperPerTurn     gHolding[1]   // amout of steps per turn (encoder settings)
#define gMessuringDuration        gHolding[2]   // duration of messurement [ms]
#define gSamplingRate             gHolding[3]   // samplingrate of measurement [Hz]
#define gRotationSpeed            gHolding[4]   // turning speed [ω/s]
#define gRotationAcceleration     gHolding[5]   // turning acceleration [ω/s²]
#define gStepperMicrosteps        gHolding[6]   // microsteps (multiplier for gHolding[0])
volatile unsigned long gStepperStepsToRun;                // amount of steps to turn 1/4


uint8_t cbMbRreadCoils         ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t cbMbWriteCoils         ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t cbcbMbReadContacts     ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t cbMbReadInputs         ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t cbMbReadHoldings       ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t writeHoldings          ( const uint8_t fc, uint16_t address, const uint16_t length );
uint8_t cbMbReadWriteRegisters ( const uint8_t fc, uint16_t address, const uint16_t length );
void    cbWriteMessurementData ();
#ifdef IS_WITHOUT_HARDWARE
void    cbMsTick();
#endif

void setup() {

  delay(2000);

  pinMode(LED_BUILTIN,  OUTPUT );
  digitalWrite(LED_BUILTIN , HIGH);

  // DRV8825 microstepping
  pinMode(DRV8825_M0_PIN,  OUTPUT );
  pinMode(DRV8825_M1_PIN,  OUTPUT );
  pinMode(DRV8825_M2_PIN,  OUTPUT );
  pinMode(DRV8825_ENA_PIN, OUTPUT );
  pinMode(DRV8825_SLP_PIN, OUTPUT );

  // DRV8825 Control
  pinMode(DRV8825_DIR_PIN,   OUTPUT );
  pinMode(DRV8825_STEP_PIN,  OUTPUT );
  pinMode(DRV8825_ENA_PIN,   OUTPUT );
  digitalWrite(DRV8825_SLP_PIN, LOW);

  mbSlave.cbVector[CB_READ_COILS]                    = cbMbRreadCoils;     // read coils
  mbSlave.cbVector[CB_WRITE_COILS]                   = cbMbRWriteCoils;    // write coils
  mbSlave.cbVector[CB_READ_DISCRETE_INPUTS]          = cbcbMbReadContacts;  // read a discrete contact
  mbSlave.cbVector[CB_READ_INPUT_REGISTERS]          = cbMbReadInputs;    // red a static input
  mbSlave.cbVector[CB_READ_HOLDING_REGISTERS]        = cbMbReadHoldings;  // red a holding
  mbSlave.cbVector[CB_WRITE_HOLDING_REGISTERS]       = writeHoldings; // write a holding
  
  DEBUGGING_PORT.begin(DEBUGGING_BAUDRATE);
  MODBUS_PORT.begin(MODBUS_BAUDRATE);
  STREAMING_PORT.begin(STREAMING_BAUDRATE);
  mbSlave.begin(MODBUS_BAUDRATE);

  stepperEngine.init();
  stepper = stepperEngine.stepperConnectToPin(DRV8825_STEP_PIN);

  if (stepper) {
    stepper->setDirectionPin(DRV8825_DIR_PIN);
    stepper->setEnablePin(DRV8825_ENA_PIN);
    stepper->setAutoEnable(false);
    stepper->enableOutputs();
    gCoils[1] = true;
  }

  stepper->setSpeedInHz( gRotationSpeed );  // the parameter is [us/step]
  stepper->setAcceleration( gRotationAcceleration );

  Timer3.initialize( 1000000 );
  Timer3.attachInterrupt( cbWriteMessurementData );
  Timer3.stop();

  // every 1ms
  Timer4.initialize(1000);
  Timer4.attachInterrupt(cbMsTick);

  DEBUGGING_PORT.println("INIT DONE");
}

void loop() {

    gCurrentEncoderValue = encoder.read();

  if (gCurrentEncoderValue != gLastEncoderValue) {
    gLastEncoderValue = gCurrentEncoderValue;
  } 

  //M// stepper direction false=CW; true=CCW 
  // if( stepper->isRunning() ){

  //   stepper->stopMove();
  //   delay(1000);
  //   if(gCoils[0]){
  //     stepper->runBackward();
  //   } else {
  //     stepper->runForward();
  //   }
  // }

  // sets the stepper direction
    if ( gStepperDirection==1 && stepper->isRunning()){
  DEBUGGING_PORT.print("0 ");
      stepper->stopMove();
      delay(100);
      stepper->runBackward();
    }

  // Enables the stepper
    if ( gStepperEnableOutputs==1 ){
        pinMode(DRV8825_SLP_PIN, LOW);
        stepper->enableOutputs();
    } else {
        if(stepper->isRunning()){
          stepper->stopMove();
        }
    stepper->disableOutputs();
    gStepperRun = false;
    }

    if ( gStepperRun ){
      pinMode(DRV8825_SLP_PIN, LOW);
      stepper->enableOutputs();
      if( gStepperDirection==0 ){
        stepper->runForward();
      } else {
        stepper->runBackward();
      }
    } else {
      // stepper->stopMove();
    }

    if ( gStepperMove ){
      stepper->enableOutputs();
      if(gCoils[0]){
        stepper->runForward();
      } else {
        stepper->runBackward();
      }
    } else {
      stepper->stopMove();
      stepper->disableOutputs();
    }

    if ( gStartMeassure ){   

      delay(10);
      if (millis() > gFinishTime){
        gMessureIsRunning = false;
        Timer3.stop();
        gStartMeassure=false;
      } else {
        Timer3.setPeriod( (unsigned long)(1.0E6/gSamplingRate) );
        gMessureIsRunning = true;
      }      
    } else {
        gStartTime  = millis();
        gFinishTime = millis() + gMessuringDuration;
    }

    if ( gSetEncoderZero ){
      gCurrentEncoderValue = 0;
      gLastEncoderValue = 0;
    }

    if(stepper->isRunning()){
      gStepperIsRunning=true;
    } else {
      gStepperIsRunning=false;
    }

    if(stepper->isStopping()){
      gStepperIsStopping=true;
    } else {
      gStepperIsStopping=false;
    }

  // slice long to short nibbles for modbus
  gInputs[0] = (uint16_t)((millis() & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[1] = (uint16_t)((millis() & 0x0000FFFF00000000UL) >> 32 );
  gInputs[2] = (uint16_t)((millis() & 0x00000000FFFF0000UL) >> 16 );
  gInputs[3] = (uint16_t)((millis() & 0x000000000000FFFFUL)       ); // MSB

  gInputs[4] = (uint16_t)((gStartTime & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[5] = (uint16_t)((gStartTime & 0x0000FFFF00000000UL) >> 32 );
  gInputs[6] = (uint16_t)((gStartTime & 0x00000000FFFF0000UL) >> 16 );
  gInputs[7] = (uint16_t)((gStartTime & 0x000000000000FFFFUL)       ); // MSB

  gInputs[8]  = (uint16_t)((gFinishTime & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[9]  = (uint16_t)((gFinishTime & 0x0000FFFF00000000UL) >> 32 );
  gInputs[10] = (uint16_t)((gFinishTime & 0x00000000FFFF0000UL) >> 16 );
  gInputs[11] = (uint16_t)((gFinishTime & 0x000000000000FFFFUL)       ); // MSB

  gInputs[12] = (uint16_t)((gCurrentEncoderPostition & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[13] = (uint16_t)((gCurrentEncoderPostition & 0x0000FFFF00000000UL) >> 32 );
  gInputs[14] = (uint16_t)((gCurrentEncoderPostition & 0x00000000FFFF0000UL) >> 16 );
  gInputs[15] = (uint16_t)((gCurrentEncoderPostition & 0x000000000000FFFFUL)       ); // MSB

  gInputs[16] = (uint16_t)((gCurrentEncoderValue & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[17] = (uint16_t)((gCurrentEncoderValue & 0x0000FFFF00000000UL) >> 32 );
  gInputs[18] = (uint16_t)((gCurrentEncoderValue & 0x00000000FFFF0000UL) >> 16 );
  gInputs[19] = (uint16_t)((gCurrentEncoderValue & 0x000000000000FFFFUL)       ); // MSB

  gInputs[20] = (uint16_t)((gCurrentStepperPosition & 0xFFFF000000000000UL) >> 48 ); // LSB
  gInputs[21] = (uint16_t)((gCurrentStepperPosition & 0x0000FFFF00000000UL) >> 32 );
  gInputs[22] = (uint16_t)((gCurrentStepperPosition & 0x00000000FFFF0000UL) >> 16 );
  gInputs[23] = (uint16_t)((gCurrentStepperPosition & 0x000000000000FFFFUL)       ); // MSB

  // amount of steps to turn
  gHolding[8]  = (uint16_t)((gStepperStepsToRun & 0xFFFF000000000000UL) >> 48 ); // LSB
  gHolding[9]  = (uint16_t)((gStepperStepsToRun & 0x0000FFFF00000000UL) >> 32 );
  gHolding[10] = (uint16_t)((gStepperStepsToRun & 0x00000000FFFF0000UL) >> 16 );
  gHolding[11] = (uint16_t)((gStepperStepsToRun & 0x000000000000FFFFUL)       ); // MSB

  mbSlave.poll();
}

#ifdef IS_WITHOUT_HARDWARE
void    cbMsTick(){
  if (gRadTimeStamp < 2*PI ){
    gRadTimeStamp += 6.336E-4;
  } else {
    gRadTimeStamp = 0;
  }
}
#endif

void cbWriteMessurementData(){

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // only print some numbers ....
  DEBUGGING_PORT.print( gHolding[15]++ );
  DEBUGGING_PORT.println( " ################" );

// CSV data
#if DATA_FORMAT==0
  #ifdef IS_WITHOUT_HARDWARE
    STREAMING_PORT.print( gStartTime );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( sin(gRadTimeStamp) );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( cos(gRadTimeStamp) );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( tan(gRadTimeStamp) );
    STREAMING_PORT.print( "\n" );
  #else 
    STREAMING_PORT.print( millis()-gStartTime );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( gCurrentStepperValue );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( gCurrentEncoderPostition );
    STREAMING_PORT.print( ";" );
    STREAMING_PORT.print( gCurrentEncoderValue );
    STREAMING_PORT.print( "\n" );
  #endif
#else if DATA_FORMAT==1
  #ifdef IS_WITHOUT_HARDWARE
    STREAMING_PORT.write( gStartTime );
    STREAMING_PORT.write( sin(gRadTimeStamp) );
    STREAMING_PORT.print( cos(gRadTimeStamp) );
    STREAMING_PORT.print( tan(gRadTimeStamp) );
    STREAMING_PORT.print( '\0');
  #else 
    STREAMING_PORT.print( millis()-gStartTime );
    STREAMING_PORT.print( gCurrentStepperValue );
    STREAMING_PORT.print( gCurrentEncoderPostition );
    STREAMING_PORT.print( gCurrentEncoderValue );
    STREAMING_PORT.print( '\0');
  #endif
#endif
}

uint8_t cbMbRreadCoils(const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;

  // sanitary check for a valid address
  if (  length > sizeof(gCoils) 
    || (address + length) >= sizeof(gCoils) ){

      return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  // address -= 1;
  for (uint8_t i = 0; i < length; i++) {
      // Write the state of the coil pin to the response buffer.
      state |= mbSlave.writeCoilToBuffer( i, gCoils[address+i] );
  }

return state;
}

uint8_t cbcbMbReadContacts (const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;
  address -= 10001;

  // sanitary check for a valid address
  if ( (length-1 >= sizeof(gContacts))
    || (address + length-1) >= sizeof(gContacts) ){
        return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  for (uint8_t i = 0; i < length; i++){
        // Write the state of the contact to the response buffer.
        state |= mbSlave.writeDiscreteInputToBuffer( i, gContacts[address + i] );
  }

  return state;
}

uint8_t cbMbReadInputs   (const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;
  address -= 30001;

  // sanitary check for a valid address
  if ( length-1 >= sizeof(gInputs)/2 
    || (address + length-1) >= (sizeof(gInputs)/2) ){
        return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  for (uint8_t i = 0; i < length; i++){
    // Write the state of the input to the response buffer.
    state |= mbSlave.writeRegisterToBuffer( i, gInputs[address + i] );
  }

  return state;
}

uint8_t cbMbReadHoldings (const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;
  address -= 40001;

  // sanitary check for a valid address
  if ( length-1 > sizeof(gHolding)/2 
    || (address + length-1) >= (sizeof(gHolding)/2) ){
        return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  for (uint8_t i = 0; i < length; i++){
    // Write the state of the input to the response buffer.
    state |= mbSlave.writeRegisterToBuffer( i, gHolding[address + i] );
  }

  return state;
}

uint8_t cbMbRWriteCoils(const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;
  // address -= 1;

  // sanitary check for valid address
  if (   length > sizeof(gCoils) 
    || ( address + length-1) >= sizeof(gCoils) ){
      return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  for (uint8_t i = 0; i < length; i++) {
      // Read the state of the coil pin from the input buffer.
       gCoils[address+i] = mbSlave.readCoilFromBuffer( i );
  }

  return state;
}

uint8_t writeHoldings(const uint8_t fc, uint16_t address, const uint16_t length){

  uint8_t state = STATUS_OK;
  address -= 40001;

  if ( (length-1 > sizeof(gHolding)/2)
    || (address + length-1) > (sizeof(gHolding)/2) ){

        return STATUS_ILLEGAL_DATA_ADDRESS;
  }

  for (uint8_t i = 0; i < length; i++) {
      // Read the state of the holding register from the input buffer.
       gHolding[address+i] = mbSlave.readRegisterFromBuffer( i );
  }

  return state;
}
/* Code written by Shreyas Fall 2017
 *  Modified by Shira Summer 2018
 *  Further Modified for single output by Shira Fall 2018
 *  
 *  Code operates in scanning mode/locking mode
 *  Scanning: sends out a ramp to the current of user-specified amplitude
 *  Locking: uses the centre-of-scan input voltage as a lockpoint, calculate PI^2 correction signal to current modulation
 *  
 *  In0 = photodiode signal
 *  Out0 = current modulation
 *  
 *  Communicates via serial - use with python cmd line program to update parameters and log data
 */

#include <EEPROM.h>
#include <analogShield.h>
#include <math.h>
#include<SPI.h>  // for ChipKit uc32 (SPI.h has to be included **after** analogShield.h)

#define UNO_ID "rb_lock\r\n"
#define ZEROV 32768 //Zero volts
#define V2P5 49512  // 2.5 V

#define STATE_LOCKING 0
#define STATE_SCANNING 1

#define ACCUMULATOR2_MAX 2000 //when accumulator is greater than this may be out of lock - code will pause accumulator
#define ACCUMULATOR1_MAX 800

#define INTEGRATOR_HOLD_TIME 500000  // microseconds - how long to pause accumulator for

struct Params {
  int ramp_amplitude;
  float gain_p, gain_i, gain_i2;
  int output_offset;
  int scan_state;
  int n_steps;
};

Params params;

int in0;
int out0, out1, out2;

bool ramp_direction;
int ramp_offset, ramp_step, ramp_step_down;
int cycle_up;
int cycle_down;
float error_signal;
float accumulator;
float accumulator2;

bool pause_accumulator;
unsigned long pause_time;

unsigned long loop_counter = 0;

bool write_to_serial_on_unlock;
bool serial_log;

void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  in0 = ZEROV;
  
  out0 = ZEROV;
  out1 = ZEROV;
  out2 = ZEROV;

  params.ramp_amplitude = 800; //bits????
  params.gain_i = 10000; // integral gain
  params.gain_p = 10000; //proportional gain - current
  params.gain_i2 = 0.01; //integral^2 gain
  
  params.output_offset = ZEROV; //offset voltage to current modulation

  
  accumulator = 0.0; //will pause accumulation if gets too big (sudden jump to laser, such as bang on optics table)
  accumulator2 = 0.0; //will not pause, used for i^2 gain
  
  params.scan_state = STATE_SCANNING; //code is in scan mode / lock mode

  params.n_steps = 800; //number of steps in one ramp up cycle
  
  processParams();
  error_signal = 0.0;

  pause_accumulator = true;
  write_to_serial_on_unlock = false; //????????????
  serial_log = false;

}

void processParams() {
  ramp_direction = true;
  ramp_offset = -params.ramp_amplitude;
  ramp_step = 2*params.ramp_amplitude/params.n_steps;
  ramp_step_down = 2*params.ramp_amplitude/params.n_steps;
  cycle_up = 0;
  cycle_down = 0;
}

void loop() {
  loop_counter += 1;
  if(Serial.available()){
    parseSerial();
    //analog.write(0,params.output_offset);
  }
    
  in0 = analog.read(0, false); //read the voltage on channel 0

  float err_curr = ((float)(in0 - ZEROV))/ZEROV; //current signal reading, in ??
  //float err_curr = ((float)(in0 - ZEROV))/6553.6; //current signal reading, in V
  error_signal = 0.95*error_signal + 0.05*err_curr; //low-passed error signal, in V
  
  out0 = ZEROV;
  out0 = params.output_offset; //add voltage offset to piezo

  // -----------------------------------------------------------------
  if(params.scan_state == STATE_SCANNING) {
    //loop_counter = 0;
    if(ramp_direction) {
      cycle_up += 1; //count how many steps up have been taken
      ramp_offset += ramp_step; //calculate the ramp up to the piezo
      if(cycle_up == params.n_steps) //stop ramping up
      {
        cycle_up = 0;
        ramp_direction = false;
      }
    }
    else {
      cycle_down += 1; //count how many steps down have been taken
      ramp_offset -= ramp_step_down; //calculate the ramp down to the piezo
      if(cycle_down ==  params.n_steps) { //stop ramping down
        cycle_down = 0;
        ramp_direction = true;    
      }
    }
    out0 += ramp_offset; //add the ramp voltage to the offset volage

  }
  // -------------------------------------------------------------------
  if(params.scan_state == STATE_LOCKING) {
  
    if(!pause_accumulator) { //...and the accumulator is not paused for a signal spike
       accumulator += params.gain_i*error_signal; //add the integral gain term to the accumulator
       accumulator2 += params.gain_i2*accumulator; //add the integral squared term to the accumualator2
    }
    if(abs(accumulator) > ACCUMULATOR1_MAX and !pause_accumulator) { //if the accumulator is too big, pause the accumulation!
      pause_accumulator = true;
      pause_time = micros();
    }
    
    if(abs(accumulator2) > ACCUMULATOR2_MAX) { //if accumulator 2 gets too big, out of lock
      // this could mean that we are out of lock... reset
      params.scan_state = STATE_SCANNING; //return to scanning mode
      if(write_to_serial_on_unlock) { //send an "out of lock" message to the serial
        Serial.write('1');
      }
    }
    if(pause_accumulator) { //if the accumulator is paused
      if(micros() - pause_time > INTEGRATOR_HOLD_TIME) { //wait until the integrator hold time has passed
        pause_accumulator = false;
        accumulator *= 0.9; //reset the accumulator
      }
    }
  
   out0 -= params.gain_p*error_signal+accumulator2+accumulator; // note the overall minus sign on the gain
  }
  // -----------------------------------------------------------------
  
  /* Output voltages are the following: 
   *  out0 = V_Poff + V_ramp    in scanning mode
   *  out0 = V_Poff - accumulator 2 - accumulator1 - Pp*error     offset, proportional gain, integral gain, integral^2 gain
   *  
   *  accumulator1: accumulate I*error
   *  accumulator2: accumulate I2*accumulator1
   */
  
  analog.write(0,out0);  //WRITE OUT THE CORRECTION SIGNALS OR SCANNING SIGNALS


  /*Write to serial for data logging in python */
  if(serial_log && loop_counter%50000 == 0) {
    Serial.print(((float)(in0 - ZEROV))/6553.6,4); //V
    Serial.print(',');
    Serial.println(((float)(out0 - ZEROV))/6553.6,4); //V
  }

}

/*
 * g - get params
 * w - write to eeprom
 * r - read from eeprom
 * i - return UNO_ID
 * s - set params
 */
void parseSerial() { //function to read and interpret characters passed from the python GUI to the serial port
  char byte_read = Serial.read();

  switch(byte_read) {
    case 'g':
      // get params, send the entire struct in one go
      Serial.write((const uint8_t*)&params, sizeof(Params));
      break;
    case 'w':
      // write to EEPROM // save new default values
      EEPROM_writeAnything(0, params);
      break;
    case 'r':
      EEPROM_readAnything(0, params); //reset to default values
      // EEPROM_readAnything(sizeof(params), logger);      
      break;
    case 'i':
      // return ID
      Serial.write(UNO_ID); //not sure..........
      break;
    case 's':
      // set params struct
      Serial.readBytes((char *) &params, sizeof(Params));
      processParams(); //update the parameters as they've been changed in the GUI
      break;
    case 'u':
      write_to_serial_on_unlock = !write_to_serial_on_unlock; //not sure..........
      break;
    case 'm':
      // toggle serial log
      serial_log = !serial_log;
      break;
  }
}

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}


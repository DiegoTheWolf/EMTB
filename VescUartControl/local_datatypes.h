#ifndef LOCAL_DATATYPES_H_
#define LOCAL_DATATYPES_H_

// Added by AC to store measured values
struct bldcMeasure {
	//7 Values int16_t not read(14 byte)
	//float temp_mos1
	//float temp_mos2
	//float temp_mos3
	//float temp_mos4
	//float temp_mos5
	//float temp_mos6
	//float temp_pcb
	float current_motor;
	float current_in;
	float duty_now;
	long rpm;
	float v_in;
	float amp_hours;
	float amp_hours_charged;
	//3 values not read (12 byte)
	//float watt_hours;
	//float watt_hours_charged;
	//long tachometer;
	long tachometerAbs;
};

//Define remote Package

struct RemoteDataStruct {
  int8_t thr;
  bool cruise;
  uint8_t _deadband;
  uint8_t _amp_fwd;   // AMPS = _amp_fwd / 2 // MAX = 127A // LSB = 0.5A
  uint8_t _amp_break; // AMPS = _amp_break / 10 // MAX = 25.5A // LSB = 0.1A
} RemoteData;

#endif
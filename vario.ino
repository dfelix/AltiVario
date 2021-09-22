/*
# vario algorithm reference
http://taturno.com/2011/10/30/variometro-la-rivincita/   
# avoid loop interference 
http://www.daqq.eu/index.php?show=prj_sanity_nullifier 
# get device voltage
http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 */

#include <Wire.h>                      // i2c library
#include <BMP085.h>                    // bmp085 library, http://code.google.com/p/bmp085driver/  
#include <Tone.h>                      // tone library, http://code.google.com/p/rogue-code/  
#include<stdlib.h>                     // contains dtostrf() -> cast float to string

///////////////////////////////////////// 
short redPin = 3;						            //  red/green Led pin (red)
short greenPin = 4;						          //  red/green Led pin (red)
short speaker_pin1 = 8;   //9 	        //  arduino speaker output -
short speaker_pin2 = 2;    //10         //  arduino speaker output +
float vario_climb_rate_start = 0.5;     //  minimum climb beeping value(ex. start climbing beeping at 0.4m/s)
float vario_sink_rate_start = -1.1;     //  maximum sink beeping value (ex. start sink beep at -1.1m/s)

#define SAMPLES_ARR 25                  //  define moving average filter array size (2->30), more means vario is less sensitive and slower, NMEA output
#define UART_SPEED 9600                 //  define serial transmision speed (9600,19200, etc...)
#define PROTOCOL 2                      //  define NMEA output: 1 = $LK8EX1, 2 = FlymasterF1, 0 = both
#define NMEA_LED_pin 6 //13             // LED NMEA out pin 
#define NMEA_OUT_per_SEC 3              // NMEA output string samples per second (1 to 20)
#define VOLUME 2                        // volume 0-no sound, 1-low sound volume, 2-high sound volume (TODO)

/////////////////////////////////////////
BMP085   bmp085 = BMP085();            
Tone     tone_out1;
Tone     tone_out2;
long     Temperature = 0;
long     Pressure = 101325;             // 1 Atm = 101325 Pa 
float    Altitude=0;
float    Battery_Vcc_mV = 0;          	//  Vcc from battery em mV
float    Battery_Vcc_V = 0;            	//  Vcc from battery em V
const float p0 = 101325;             		//  Pressure at sea level (Pa)
unsigned long timestamp01 = millis();		//  save initial timestamp
unsigned long timestamp02 = millis();		//  timer for temperature check (checkTempEveryMs)
unsigned long timestamp03 = millis();		//  timer for NMEA_OUT
unsigned long timestamp04 = millis();		//  timer for battery check (checkBattEveryMs)
int      my_temperature = 1;
char     altitude_arr[6];               // array for float to string cast
char     vario_arr[6];                  // array for float to string cast
char     batt_arr[6];
char     pressure_arr[10];

int      samples=40;
int      maxsamples=50;
float    alt[51];
float    tim[51];
float    beep;
float    Beep_period;
static long k[SAMPLES_ARR];
int 	  checkTempEveryMs=1000;		 // temperature check every ... seconds
int     checkBattEveryMs=10000;		// battery check every ... seconds

static long Averaging_Filter(long input);
static long Averaging_Filter(long input) // moving average filter function
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    k[i] = k[i+1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}

void play_welcome_beep()                 //play only once welcome beep after turning on arduino vario
{
  for (int aa=300;aa<=1500;aa=aa+100)
  {
    tone_out1.play(aa,200);                       // play beep on pin (note,duration)
   if (VOLUME==2){ tone_out2.play(aa+5,200);}     // play beep on pin (note,duration)
    delay(100);
  }
  for (int aa=1500;aa>=100;aa=aa-100)
  {
    tone_out1.play(aa,200);                       // play beep on pin (note,duration)
   if (VOLUME==2){ tone_out2.play(aa+5,200);}     // play beep on pin (note,duration)
    delay(100);
  }
}



long readVcc()
{ 
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void setup()                // setup() function to setup all necessary parameters before we go to endless loop() function
{
  Serial.begin(UART_SPEED);       // set up arduino serial port
  Wire.begin();             // lets init i2c protocol
  if (VOLUME==1){
  tone_out1.begin(speaker_pin1);       // piezo speaker output pin8 -
  pinMode(speaker_pin2, OUTPUT);      // set pin for output NMEA LED blink;
  digitalWrite(speaker_pin2, LOW); 
  } 
  else if (VOLUME==2){
  tone_out1.begin(speaker_pin1);       // piezo speaker output pin8 -
  tone_out2.begin(speaker_pin2);       // piezo speaker output pin9 +
  }

 pinMode(redPin, OUTPUT);        // sets the digital pin as output
 pinMode(greenPin, OUTPUT);      // sets the digital pin as output 
 digitalWrite(greenPin, HIGH);
 digitalWrite(redPin, HIGH); 


  bmp085.init(MODE_ULTRA_HIGHRES, p0, false); 
                            // BMP085 ultra-high-res mode, 101325Pa = 1013.25hPa, false = using Pa units
                            // this initialization is useful for normalizing pressure to specific datum.
                            // OR setting current local hPa information from a weather station/local airport (QNH).
 //play_welcome_beep();      //everything is ready, play "welcome" sound
 pinMode(NMEA_LED_pin, OUTPUT);      // set pin for output NMEA LED blink;

}
float nmea_vario_cms =0;
float nmea_time_s=0;
float nmea_alt_m=0;
float nmea_old_alt_m=0;

void loop(void)
{
  float time=millis();              //take time, look into arduino millis() function
  float vario=0;
  float N1=0;
  float N2=0;
  float N3=0;
  float D1=0;
  float D2=0;
  bmp085.calcTruePressure(&Pressure);                                   //get one sample from BMP085 in every loop
  long average_pressure = Averaging_Filter(Pressure);                   //put it in filter and take average, this averaging is for NMEA output
  Altitude = (float)44330 * (1 - pow(((float)Pressure/p0), 0.190295));  //take new altitude in meters from pressure sample, not from average pressure
 //
 nmea_alt_m = (float)44330 * (1 - pow(((float)average_pressure/p0), 0.190295));
 if ((millis() >= (nmea_time_s+1000))){
 nmea_vario_cms = ((nmea_alt_m-nmea_old_alt_m))*100; 
 nmea_old_alt_m = nmea_alt_m;
 nmea_time_s = millis();
 }

 // algorimo do vario
  for(int cc=1;cc<=maxsamples;cc++){
    alt[(cc-1)]=alt[cc]; //avança com o valor da "altitude" no array
    tim[(cc-1)]=tim[cc]; //avança com o valor do "time" no array
  }; 
  alt[maxsamples]=Altitude; //coloca a "altitude" no fim do array
  tim[maxsamples]=time; //coloca o "time" no fim do array
  float stime=tim[maxsamples-samples];
  for(int cc=(maxsamples-samples);cc<maxsamples;cc++){
    N1+=(tim[cc]-stime)*alt[cc];
    N2+=(tim[cc]-stime);
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  };

 vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);
 if ((time-beep)>Beep_period)                          // TODO: eliminar o segundo piezo 
  {
    beep=time;
    if (vario>vario_climb_rate_start && vario<=10 )
    {
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario))); //sobe, logo beeps rapidos
        case 2:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario))); //sobe, logo beeps rapidos
          tone_out2.play((1406+(200*vario)),420-(vario*(20+vario)));
      }               
    } else if (vario >10) 
    {
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=160;
          tone_out1.play(3450,120);
        case 2:
          Beep_period=160;
          tone_out1.play(3450,120);
          tone_out2.play(3456,120);
      }               
    } else if (vario< vario_sink_rate_start){

      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=200;
          tone_out1.play(300,340);
        case 2:
          Beep_period=200;
          tone_out1.play(300,340);
          tone_out2.play(320,340);
      }               
    }
  }

  if (millis() >= (timestamp02+checkTempEveryMs))      //guarda temperatura e nivel bateria a casa segundo
  {
    bmp085.getTemperature(&Temperature);
    my_temperature = Temperature/10;
    timestamp02 = millis();
  }

  if (millis() >= (timestamp04+checkBattEveryMs))      //every 10 second get battery level
  {
    Battery_Vcc_mV = readVcc();                      //get voltage in milivolts
    Battery_Vcc_V =(float(Battery_Vcc_mV)/1000);     //get voltage in volts
    timestamp04 = millis();

    //Serial.println(Battery_Vcc_V);
    
    if(Battery_Vcc_V > 3.60)
    {
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, LOW); 
    }
    if(Battery_Vcc_V > 3.40 && Battery_Vcc_V < 3.60)
    {
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, HIGH); 
    }
    if(Battery_Vcc_V <= 3.40)
    {
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, HIGH); 
    }
  }

  

//LK8000
//  $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
//	pressure in hPascal:hPA*100 (example for 1013.25 becomes 101325) no padding (987.25 becomes 98725, NOT 098725)
// If no pressure available, send 999999 (6 times 9)
// If pressure is available, field 1 altitude will be ignored
// Field 1, altitude in meters, relative to QNH 1013.25
//  If raw pressure is available, this value will be IGNORED (you can set it to 99999
// but not really needed)!(if you want to use this value, set raw pressure to 999999)
//  This value is relative to sea level (QNE). We are assuming that currently at 0m
// altitude pressure is standard 1013.25.If you cannot send raw altitude, then send
//  what you have but then you must NOT adjust it from Basic Setting in LK.
//  Altitude can be negative. If altitude not available, and Pressure not available, set Altitude
//  to 99999. LK will say "Baro altitude available" if one of fields 0 and 1 is available.
//  Field 2, vario in cm/s
//  If vario not available, send 9999. Value can also be negative.
//  Field 3, temperature in C , can be also negative.If not available, send 99
//  Field 4, battery voltage or charge percentage.Cannot be negative.If not available, send 999.
// Voltage is sent as float value like: 0.1 1.4 2.3 11.2. To send percentage, add 1000.
// Example 0% = 1000. 14% = 1014 . Do not send float values for percentages.
// Percentage should be 0 to 100, with no decimals, added by 1000!
 
 if ((millis() >= (timestamp03+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 1))       
  {
    int battery_percentage = map(Battery_Vcc_mV, 2500, 4300, 1, 100);  
    String str_out =                                                                 
      String("LK8EX1"+String(",")+String(average_pressure,DEC)+String(",")+String(dtostrf(Altitude,0,0,altitude_arr))+String(",")+
      String(dtostrf(nmea_vario_cms,0,0,vario_arr))+String(",")+String(my_temperature,DEC)+String(",")+String(battery_percentage+1000)+String(","));
    unsigned int checksum_end,ai,bi;
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end,HEX);      //print calculated checksum on the end of the string in HEX
    timestamp03 = millis();
    if (digitalRead(NMEA_LED_pin)== HIGH) {digitalWrite(NMEA_LED_pin, LOW);} else {digitalWrite(NMEA_LED_pin, HIGH);}
   }

  //Flymaster F1 
  //$VARIO,fPressure,fVario,Bat1Volts,Bat2Volts,BatBan k,TempSensor1,TempSensor2*CS
  
  //Where:
  //fPressure = the absolute atmospheric pressure, converting to altitude use the following:
  //fBarAltitude = (1 - pow(fabs(fPressure / fQNH), 0.190284)) * 44307,69;        
  //fVario = the variometer in decimeters per second
  //Bat1Volts = the voltage of the battery in bank 1
  //Bat2Volts = the voltage of the battery in bank 2
  //BatBank = the battery bank in use.
  //TempSensor1 = temperature in ?C of external wireless sensor 1
  //TempSensor2 = temperature in ?C of external wireless sensor 2
  //CS = the standard NMEA checksum.

  if ((millis() >= (timestamp03+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 2)) 
  {   
    //signed
    /*
    String str_out =
    String("VARIO,"+                                      //$VARIO
    String(dtostrf(((float)average_pressure/100),2,2,pressure_arr))+ //fPressure
    String(",")+
    String(dtostrf((nmea_vario_cms/10),2,2,vario_arr))+   //fVario
    String(",")+                                   
    String(dtostrf((Battery_Vcc_V),2,2,batt_arr))+  //Bat1Volts
    String(",0,1")+                                   //Bat2Volts,BatBan k,
    String(",")+
    String(my_temperature,DEC)+                           //TempSensor1
    String(",0"));                                        //TempSensor2
    */
    //unsigned
    String str_out =
    String("VARIO,"+                                      //$VARIO
    String(dtostrf(((float)average_pressure/100),0,0,pressure_arr))+ //fPressure
    String(",")+
    String(dtostrf((nmea_vario_cms/10),2,2,vario_arr))+   //fVario
    String(",")+                                   
    String(dtostrf((Battery_Vcc_V),2,2,batt_arr))+  //Bat1Volts
    String(",0,1")+                                   //Bat2Volts,BatBan k,
    String(",")+
    String(my_temperature,DEC)+                           //TempSensor1
    String(",0"));                                        //TempSensor2

		
    unsigned int checksum_end,ai,bi;
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    Serial.print("$");
    Serial.print(str_out);
    Serial.print("*");
    Serial.println(checksum_end,HEX);      //checksum
    
    timestamp03 = millis();
    if (digitalRead(NMEA_LED_pin)== HIGH) 
    {
      digitalWrite(NMEA_LED_pin, LOW);
    }
    else
    {
      digitalWrite(NMEA_LED_pin, HIGH);
    }
  }
}


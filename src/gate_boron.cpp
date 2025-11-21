/* 
 * Project gate_boron
 * Author: Matt Stephens
 * Date: 10-06-2025
 * Fixed issues with wiegand library in preparation for adding package box contacts
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "application.h"
#include "BeaconScanner.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

//load wiegand library for keycards
//below was wiegand.h
class WIEGAND {

public:
	WIEGAND();
	void begin();
	void begin(int pinD0, int pinIntD0, int pinD1, int pinIntD1);
	bool available();
	unsigned long getCode();
	int getWiegandType();
	
private:
	static void ReadD0();
	static void ReadD1();
	static bool DoWiegandConversion ();
	static unsigned long GetCardId (volatile unsigned long *codehigh, volatile unsigned long *codelow, char bitlength);
	
	static volatile unsigned long 	_cardTempHigh;
	static volatile unsigned long 	_cardTemp;
	static volatile unsigned long 	_lastWiegand;
	static volatile int				_bitCount;	
	static int				_wiegandType;
	static unsigned long	_code;
};

//#include "wiegand.h" --- below was wiegand.cpp
volatile unsigned long WIEGAND::_cardTempHigh=0;
volatile unsigned long WIEGAND::_cardTemp=0;
volatile unsigned long WIEGAND::_lastWiegand=0;
unsigned long WIEGAND::_code=0;
volatile int WIEGAND::_bitCount=0;	
int WIEGAND::_wiegandType=0;

WIEGAND::WIEGAND()
{
}

unsigned long WIEGAND::getCode()
{
	return _code;
}

int WIEGAND::getWiegandType()
{
	return _wiegandType;
}

bool WIEGAND::available()
{
	bool ret;
    noInterrupts();
	ret=DoWiegandConversion();
	interrupts();
	return ret;
}

void WIEGAND::begin()
{
  begin(D2,D2,D3,D3);
}

void WIEGAND::begin(int pinD0, int pinIntD0, int pinD1, int pinIntD1)
{
	_lastWiegand = 0;
	_cardTempHigh = 0;
	_cardTemp = 0;
	_code = 0;
	_wiegandType = 0;
	_bitCount = 0;  
	pinMode(pinD0, INPUT);					// Set D0 pin as input
	pinMode(pinD1, INPUT);					// Set D1 pin as input
	attachInterrupt(pinIntD0, ReadD0, FALLING);	// Hardware interrupt - high to low pulse
	attachInterrupt(pinIntD1, ReadD1, FALLING);	// Hardware interrupt - high to low pulse
}

void WIEGAND::ReadD0 ()
{
	_bitCount++;				// Increament bit count for Interrupt connected to D0
	if (_bitCount>31)			// If bit count more than 31, process high bits
	{
		_cardTempHigh |= ((0x80000000 & _cardTemp)>>31);	//	shift value to high bits
		_cardTempHigh <<= 1;
		_cardTemp <<=1;
	}
	else
	{
		_cardTemp <<= 1;		// D0 represent binary 0, so just left shift card data
	}
	_lastWiegand = millis();	// Keep track of last wiegand bit received
}

void WIEGAND::ReadD1()
{
	_bitCount ++;				// Increment bit count for Interrupt connected to D1
	if (_bitCount>31)			// If bit count more than 31, process high bits
	{
		_cardTempHigh |= ((0x80000000 & _cardTemp)>>31);	// shift value to high bits
		_cardTempHigh <<= 1;
		_cardTemp |= 1;
		_cardTemp <<=1;
	}
	else
	{
		_cardTemp |= 1;			// D1 represent binary 1, so OR card data with 1 then
		_cardTemp <<= 1;		// left shift card data
	}
	_lastWiegand = millis();	// Keep track of last wiegand bit received
}

unsigned long WIEGAND::GetCardId (volatile unsigned long *codehigh, volatile unsigned long *codelow, char bitlength)
{
	unsigned long cardID=0;

	if (bitlength==26)								// EM tag
		cardID = (*codelow & 0x1FFFFFE) >>1;

	if (bitlength==34)								// Mifare 
	{
		*codehigh = *codehigh & 0x03;				// only need the 2 LSB of the codehigh
		*codehigh <<= 30;							// shift 2 LSB to MSB		
		*codelow >>=1;
		cardID = *codehigh | *codelow;
	}
	return cardID;
}

char translateEnterEscapeKeyPress(char originalKeyPress) {
    switch(originalKeyPress) {
        case 0x0b:        // 11 or * key
            return 0x0d;  // 13 or ASCII ENTER

        case 0x0a:        // 10 or # key
            return 0x1b;  // 27 or ASCII ESCAPE

        default:
            return originalKeyPress;
    }
}

bool WIEGAND::DoWiegandConversion ()
{
	unsigned long cardID;
	unsigned long sysTick = millis();
	
	if ((sysTick - _lastWiegand) > 25)								// if no more signal coming through after 25ms
	{
		if ((_bitCount==26) || (_bitCount==34) || (_bitCount==8) || (_bitCount==4)) 	// bitCount for keypress=4 or 8, Wiegand 26=26, Wiegand 34=34
		{
			_cardTemp >>= 1;			// shift right 1 bit to get back the real value - interrupt done 1 left shift in advance
			if (_bitCount>32)			// bit count more than 32 bits, shift high bits right to make adjustment
				_cardTempHigh >>= 1;	

			if((_bitCount==26) || (_bitCount==34))		// wiegand 26 or wiegand 34
			{
				cardID = GetCardId (&_cardTempHigh, &_cardTemp, _bitCount);
				_wiegandType=_bitCount;
				_bitCount=0;
				_cardTemp=0;
				_cardTempHigh=0;
				_code=cardID;
				return true;				
			}
			else if (_bitCount==8)		// keypress wiegand with integrity
			{
				// 8-bit Wiegand keyboard data, high nibble is the "NOT" of low nibble
				// eg if key 1 pressed, data=E1 in binary 11100001 , high nibble=1110 , low nibble = 0001 
				char highNibble = (_cardTemp & 0xf0) >>4;
				char lowNibble = (_cardTemp & 0x0f);
				_wiegandType=_bitCount;					
				_bitCount=0;
				_cardTemp=0;
				_cardTempHigh=0;
				
				if (lowNibble == (~highNibble & 0x0f))		// check if low nibble matches the "NOT" of high nibble.
                {
                    _code = (int)translateEnterEscapeKeyPress(lowNibble);
					return true;
				}

                // TODO: Handle validation failure case!
			}
            else if (4 == _bitCount) {
                // 4-bit Wiegand codes have no data integrity check so we just
                // read the LOW nibble.
                _code = (int)translateEnterEscapeKeyPress(_cardTemp & 0x0000000F);

                _wiegandType = _bitCount;
                _bitCount = 0;
                _cardTemp = 0;
                _cardTempHigh = 0;

                return true;
            }
		}
		else
		{
			// well time over 25 ms and bitCount !=8 , !=26, !=34 , must be noise or nothing then.
			_lastWiegand=sysTick;
			_bitCount=0;			
			_cardTemp=0;
			_cardTempHigh=0;
			return false;
		}	
	}
	else
		return false;
  return false;
}
// END WIEGAND IMPORTED CODE LIBRARY -----------------------------------------------------------------------------------------------------------------
WIEGAND wg;

//beaconscanner items
//unsigned long scannedTime = 0;
int lastpub_pkg = System.millis(); //minute of last publish
String pkgboxstatus = "Unknown";
BleAddress pkgboxbeacon("DD:88:00:00:0B:3B", BleAddressType::PUBLIC);

//base gate code items
String gatepos = "unknown"; //gate position as text (open/closed/unknown)
bool prevpos = true; //previous gate position to detect a change
String command = ""; //used for gate command function
int gatebool = true; //gate position as 0/1 (0=closed, 1=open)
int alarmstate = 0; //state of alarm on last run
int held_open = 0; //stores whether gate open button is being held down;
int return_code;
int curtime; //current time in minutes, min publish for gate position every 5 mins
int lastpub = System.millis(); //ms of last publish
int holdend = System.millis(); //ms when to close held gate

//remote reset code
#define DELAY_BEFORE_REBOOT 2000
unsigned int rebootDelayMillis = DELAY_BEFORE_REBOOT;
unsigned long rebootSync = millis();
bool resetFlag = false;

//handle cloud requests
int gate_command(String rec_command)
{
  command = rec_command;
  return_code = 9;
  //regular open
  if (command == "open") {
      digitalWrite(7,HIGH);
      delay(500);
      digitalWrite(7,LOW);
      return_code = 1;
  }
  //hold open commands of different lengths
  if (command == "hold_open_30m") {
      digitalWrite(7,HIGH);
      held_open = 1;
      holdend = System.millis() + 30*60*1000;
      return_code = 2;
  } //30 minutes
  if (command == "hold_open_1hr") {
      digitalWrite(7,HIGH);
      held_open = 1;
      holdend = System.millis() + 60*60*1000;
      return_code = 2;
  } //1 hour
  if (command == "hold_open_4hr") {
      digitalWrite(7,HIGH);
      held_open = 1;
      holdend = System.millis() + 4*60*60*1000;
      return_code = 2;
  } //4 hours
  if (command == "hold_open_8hr") {
      digitalWrite(7,HIGH);
      held_open = 1;
      holdend = System.millis() + 8*60*60*1000;
      return_code = 2;
  } //8 hours
  //close immediately
  if (command == "close") {
      if (held_open == 1) {
          digitalWrite(7,LOW);
          held_open = 0;
          Particle.publish("hgholdremain","0");
          delay(1000);
      }
      digitalWrite(5,HIGH);
      delay(500);
      digitalWrite(5,LOW);
      return_code = 0;
  }
return return_code;
}
//  Remote Reset Function
int cloudResetFunction(String command) {
  resetFlag = true;
  rebootSync = millis();
  return 0;
}

// setup() runs once, when the device is first turned on.
void setup() {
    // Put initialization like pinMode and begin functions here.
    pinMode(5,OUTPUT); //close relay
    pinMode(7,OUTPUT); //open relay
    pinMode(15,INPUT_PULLDOWN); //alarm input - pull down (hot=on) | A4 | middle two terminals on board
    pinMode(17,INPUT); //keypad input Weigand DO | A2 | bottom two terminals on board
    pinMode(18,INPUT); //keypad input Weigand D1 | A2 | bottom two terminals on board
    pinMode(16,INPUT_PULLUP); //position input - pull up (closed=high) | A3 | top two terminals on board
    pinSetDriveStrength(5,DriveStrength::HIGH);
    pinSetDriveStrength(7,DriveStrength::HIGH);
    Particle.function("gate_command", gate_command);
    Particle.function("reset", cloudResetFunction);
    Particle.publish("hgalarm","0"); //set initial alarm condition
    wg.begin(D17,D17,D18,D18); //start weigand reading
    //BLE and Beacon setup
    BLE.on();
    Scanner.startContinuous(SCAN_IBEACON);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  curtime = System.millis();

  // handling open/closed indication
  prevpos = gatebool;
  
  //manage held open gate
  if (held_open == 1 && gatebool) {
    //gate is in held open mode and gate is open (true)
    if (curtime >= holdend) {
      //close da gate!
      int autoclose = gate_command("close");
    }
  }
  //check inputs
  if (digitalRead(16)==1) {
    //pull-up resistor is reading, contact is open; gate is closed
    gatebool = false;
    gatepos = "closed";
  } 
  if(digitalRead(16)==0) {
    //shorted to com by gate dry contact, gate is open
    gatebool = true;
    gatepos = "open";
  }
  
  if (prevpos!=gatebool) {
    //the position has been changed, send a publish immediately to cloud
    Particle.publish("hgpos",gatepos);
    //Particle.publish("Debug", "Gate position changed", 300, PRIVATE);
  }
  
  //handling alarm indication
  if (digitalRead(15)==1 && alarmstate==0) {
      //hard shutdown of main board
      Particle.publish("hgalarm","1");
  } 
  if (digitalRead(15)==0 && alarmstate==1) {
      //clear alarm
      Particle.publish("hgalarm","0");
  }
  
  //scheduled publishes
  if (curtime-lastpub>=1000*60*5) {
    //time based publish (heartbeat)
    Particle.publish("hgpos",gatepos);
    //publish remaining hold time
    if (held_open == 1) {
      Particle.publish("hgholdremain",String(((holdend - curtime)/(1000*60))));
    }
    //package box status when clear
    Particle.publish("packagebox",pkgboxstatus);
    pkgboxstatus = "Unknown"; //set to unknown after publish
    lastpub = curtime;
  }
  
    //  Remote Reset Function
  if ((resetFlag) && (millis() - rebootSync >=  rebootDelayMillis)) {
    // do things here  before reset and then push the button
    Particle.publish("Debug", "Remote Reset Initiated");
    System.reset();
  }
  //wiegand read - pass for records
  if(wg.available())
  {
      //   Serial.print("Wiegand HEX = ");
      //   Serial.print(wg.getCode(),HEX);
      //   Serial.print(", DECIMAL = ");
      //   Serial.print(wg.getCode());
      //   Serial.print(", Type W");
      //   Serial.println(wg.getWiegandType()); 
      Particle.publish("keypad", String(wg.getCode()));   
  }
  //BLE beacon scan
  Scanner.loop();
  Vector<iBeaconScan> beacons = Scanner.getiBeacons();
    while(!beacons.isEmpty())
    {
      iBeaconScan beacon = beacons.takeFirst();
      //Log.info("Address: %s, major: %u, minor: %u", beacon.getAddress().toString().c_str(), beacon.getMajor(), beacon.getMinor());
      if (strcmp(beacon.getAddress().toString(),pkgboxbeacon.toString()) == 0) {
        //found the intended beacon
        if (strcmp(beacon.getUuid(),"7777772E-6B6B-6D63-6E2E-636F6D000002") == 0) {
          //found the motion UUID
          pkgboxstatus = "Detected";
          if (System.millis() - lastpub_pkg >= 1000*60) {
            Particle.publish("packagebox",pkgboxstatus);
            lastpub_pkg = System.millis();
          }
        } else if (strcmp(beacon.getUuid(),"426C7565-4368-6172-6D42-6561636F6E73") == 0) {
          //found standard state, wait on scheduled publish interval
          pkgboxstatus = "Clear";
        }
      }
    } //end while beacons
} //endloop
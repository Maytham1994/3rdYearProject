#include <JeeLib.h>
#include <Ports.h>

// Joystick values structure
struct Joy_Vals
{
  int JoyR_L;
  int JoyD_U;
};


Port port1 = Port(1); // Up-Down Value Port
Port port2 = Port(2); // Right-Left Value Port
Port port3 = Port(3); // Vehicle_Status Button

void setup() { 
 // Begin RF Transmission on Channel 69
 rf12_initialize(2, RF12_915MHZ, 69);
 
 // Setting the AIO pin on ports 1 and 2 to input, and setting the DIO pin on port 3 to input (Vehicle_Status Button)
 port1.mode2(INPUT); 
 port2.mode2(INPUT);
 port3.mode(INPUT);
}

// Joystick values reader, return a Joy_Vals structure
struct Joy_Vals Joy_Get()
{
	// Joystick values temp variables
	int JoyD_U=0,JoyR_L=0;
	// loop variable
	int i=0;
	for(;i<20;i++){
		// reading in the Joystick values 20 times and finding the sum
		JoyD_U += port1.anaRead(); // read from AIO of the “port”
		JoyR_L += port2.anaRead(); // read from AIO of the “port"
	}
  
	// creating the temporary structure
	struct Joy_Vals Joy_Values;
	// averaging the read values and saving in the structure
	Joy_Values.JoyD_U = (JoyD_U/20);
	Joy_Values.JoyR_L = (JoyR_L/20);
   
	// returning the structure
	return Joy_Values;
}

/*	This function is used in the remote to send data to the vehicle
*	This function is not used on the vehicle
*	
*	vehicle_status: status of the vehicle (1: remote, 0: autonomous)
*	AIN1, AIN2, BIN1, BIN2; motor states; forward/backward for motors 1 and 2 
*	r_motor, l_motor: right and left motor PWM values
*/
void jeenode_send(bool vehicle_status, bool AIN1, bool AIN2, bool BIN1, bool BIN2, byte r_motor,byte l_motor)
{
  // array of 3 bytes 
  byte send_msg[3];
  
  // setting the vehicle_status in the transmission
  if (vehicle_status == true)
    send_msg[0] = 0xF0;
  else
    send_msg[0] = 0x00;
    
  send_msg[0] |= AIN2 << 3;
  send_msg[0] |= AIN1 << 2;
  send_msg[0] |= BIN1 << 1;
  send_msg[0] |= BIN2;
   
  // setting the motor PWMs in the transmission
  send_msg[1] = r_motor;
  send_msg[2] = l_motor;
    
   // checking if sending can be done
   while(!rf12_canSend()) // Is the air busy, or can you send?
     rf12_recvDone(); //if you cant send, complete any receiving that you      
                                  // are doing
   // sending the data
   rf12_sendStart(0, &send_msg, sizeof(byte) * 3); // give address to payload, and 
						   // the payload size is 5 which is 
						  // the size of the packet

}

void loop() { 
	// Temporary variables
	int JoyD_U=0;
	int JoyR_L=0;
	bool AN1 = 0;
	bool AN2 = 0;
	bool BN1 = 0;
	bool BN2 = 0;
	byte ValueR = 0;
	byte ValueL = 0;
	bool remote = 0;
 
	// creating a structure, reading the Joystick values and putting them in the temporary variables
	struct Joy_Vals values = Joy_Get();
	JoyD_U = values.JoyD_U;
	JoyR_L = values.JoyR_L;
        
	// reading the vehicle status button (1 for remote, 0 for autonomous)
	remote = port3.digiRead();
 
	if(JoyD_U > 550){
		// The vehicle has a forward movement
		AN1=true;
		BN1=true;
	}else if(JoyD_U < 474){
		// The vehicle has a backward movement
		AN2=true;
		BN2=true;
	}else if(JoyR_L > 550){
		// The vehicle is doing a 360 to the left
		AN2=true;
		BN1=true;
	}else if(JoyR_L < 474){
		// The vehicle is doing a 360 to the right
		AN1=true;
		BN2=true;
	}
 
 
	if(JoyR_L > 950){
		// The vehicle is doing a hard left turn
		ValueL = 110;
		ValueR = 255;
	}else if(JoyR_L > 850){
		// The vehicle is doing a slower left turn
		ValueL = 255;
		ValueR = 140;
	}else if(JoyR_L > 750){
		// The vehicle is doing a slower left turn
		ValueL = 255;
		ValueR = 160;
	}else if(JoyR_L > 650){
		// The vehicle is doing a slower left turn
		ValueL = 255;
		ValueR = 180;
	}else if(JoyR_L > 550){
		// The vehicle is doing slow left turn
		ValueL = 255;
		ValueR = 210;
	}else if(JoyR_L < 74){
		// The vehicle is doing a hard right turn
		ValueL = 255;
		ValueR = 110;
	}else if(JoyR_L < 174){
		// The vehicle is doing a slower right turn
		ValueL = 255;
		ValueR = 140;
	}else if(JoyR_L < 274){
		// The vehicle is doing a slower right turn
		ValueL = 255;
		ValueR = 160;
		}else if(JoyR_L < 374){
		// The vehicle is doing a slower right turn
		ValueL = 255;
		ValueR = 180;
	}else if(JoyR_L < 474){
		// The vehicle is doing slow right turn
		ValueL = 255;
		ValueR = 210;
	}else{
		// The vehicle is not doing any turns, therefore either forward or backward
		ValueL = 255;
		ValueR = 255;
	}
 
	// Transmitting the data to the vehicle
	jeenode_send(remote,AN1,AN2,BN1,BN2,ValueR,ValueL);
}

// including the JeeLib (JeeNode library), Ports (JeeNode ports library), 
// and the Wire I2C library
#include <JeeLib.h>
#include <Ports.h>
#include <Wire.h>

/* The A2,A1,A0 pins are connected to ground, therefore the address is 0x20
* The address is 0b0100[A2][A1][A0]
*/
#define IOAddress 0x20
// Addresses for the GP0 and GP1 registers in the MCP23016
#define GP0	0x00
#define GP1	0x01
// When set to 1, the port is INPUT, when set to 0, the port is OUTPUT.
#define IODIR0 0x06
#define IODIR1 0x07

// Setting all the pins in a port to either output, or input
#define ALL_OUTPUT	0x00
#define ALL_INPUT	0xFF

// IR sensor structure for the left and the right IR sensors
struct IR_Vals
{
  int IR_L;
  int IR_R;
};

// Structure that holds the current directions of the I/O expander ports
struct IOState{
  // These variables hold the current state of each of the two ports, directions of the I/O
  unsigned char GP0CurrentState;
  unsigned char GP1CurrentState;
};

// Structure that holds the current values of the I/O expander's output
struct IOPortValues{
  // These variables hold the current state of each of the two ports, directions of the I/O
  unsigned char IOPort0;
  unsigned char IOPort1;
};

// Setting up the 4 JeeNode ports
Port port1 = Port(1);
Port port2 = Port(2);
Port port3 = Port(3);
Port port4 = Port(4);

// This function start the transmission I2C protocol
void Start_I2C(){
	Wire.begin();
}

/*
*	This function sets the direction of the two I/O ports
*	Parameters: GP0State and GP1State are the desired states for the two ports\
*	This function updates the two current state parameters
*/
struct IOState MCP_SetDirection(unsigned char GP0State, unsigned char GP1State){
	IOState CurrentState;
        // Update the two CurrentState parameters
	CurrentState.GP0CurrentState = GP0State;
	CurrentState.GP1CurrentState = GP1State;
	// Start I2C transmission with the device
	Wire.beginTransmission(IOAddress);
	// Selecting IODIR0 register to write to
	Wire.write(IODIR0);
	// Setting the state for port0
	Wire.write(GP0State);
	// Selecting IODIR1 register to write to
	Wire.write(IODIR1);
	// Setting the state for port1
	Wire.write(GP1State);

        Wire.write(GP0);
	// Setting the state for port0
	Wire.write(0x00);
	// Selecting IODIR1 register to write to
	Wire.write(GP1);
	// Setting the state for port1
	Wire.write(0x00);


	// Ending I2C transmission
	Wire.endTransmission();

        return CurrentState;
}

/*
*	This function writes data to the ports on the I/O expander
*	Parameters: port (true for port1, false for port0), data: holds the data desired to be writted
*/
void MCP_WritePort(bool port, unsigned char data){
	// Checking if the desired port is 0 or 1
	if(port){
		// Start the transmission with the device
		Wire.beginTransmission(IOAddress);
		// Select port1 to write to
		Wire.write(GP1);
		// Write the data to port1
		Wire.write(data);
		// End the transmission
		Wire.endTransmission();
	}else{
		// Start the transmission with the device
		Wire.beginTransmission(IOAddress);
		// Select port0 to write to
		Wire.write(GP0);
		// Write the data to port0
		Wire.write(data);
		// End the transmission
		Wire.endTransmission();
	}
}

/*
*	This function reads the data from ports 0 and 1 and returns an array with the data
*	Parameters: port (true for port1, false for port0)
*	Return: array with data from port0 and port1
*/
struct IOPortValues MCP_ReadPorts(bool port){
        IOPortValues result;
	// Temporary array to hold he values
	unsigned char array[2];
	// Temporarily save 0 in both values
	array[0]=0x00;
	array[1]=0x00;
	// Start transmission with the device
	Wire.beginTransmission(IOAddress);
	// Request the device for the number of bytes, if port1 then the bytes are 2, if port0 then the bytes are 1
	Wire.requestFrom(IOAddress, port+1);
	// Save the data in the array
	for(int i = 0; Wire.available(); i++){
		array[i] = Wire.read();
	}
        result.IOPort0 = array[0];
        result.IOPort1 = array[1];
	// Return the array
	return result;
}

/*
*	This function is specific to the H-Bridge used for Group Hotel in ELEC3907 Project
*	Parameters: AIN1,AIN2,BIN1,BIN2 are specific for the H-Bridge
*/
void Motor_Directions(bool AIN1, bool AIN2, bool BIN1, bool BIN2){
	// Read the current values in the port
	IOPortValues values = MCP_ReadPorts(0);
	// Save the port in a temporary direction variable
	unsigned char directions = values.IOPort1;
	// Temporary Direction Variables
	unsigned char temp1 = 0x0F;
	unsigned char temp2 = 0x00;
	// Set AIN1
	temp1 |= AIN2 << 7;
	temp2 |= AIN2 << 7;
	// Set AIN2
	temp1 |= AIN1 << 6;
	temp2 |= AIN1 << 6;
	// Set BIN1
	temp1 |= BIN1 << 5;
	temp2 |= BIN1 << 5;
	// Set BIN2
	temp1 |= BIN2 << 4;
	temp2 |= BIN2 << 4;
	
	// ANDing the current values with temp1
	directions &= temp1;
	// ORing the current values with temp2
	directions |= temp2;
	
	/*
	*	Code above does this;
	*	If current output is 0b10101010, and the desired 4 bits are 0b1100
	*	Set temp1 to 0b11001111, and temp2 to 0b11000000
	*	Then when ANDing 0b10101010 & 0b11001111 = 0b10001010
	*	Then ORing 0b10001010 | 0b11000000 = 11001010
	*	Therefore direction is now 0b11001010, therefore only the 4 bits changed and the rest remained the same
	*/
	// Call the write function to write the data to port0
	MCP_WritePort(0, directions);
}

// hard right turn function, this function turns the vehicle right with a hard turn
void turnHardRight() {
	// Setting the directions of the motors with the I/O expander to forward
	Motor_Directions(HIGH,LOW,HIGH,LOW);
	// setting the left motor's PWM to be at a higher rate than the right one
	port3.anaWrite(255);
	// setting the right motor's PWM to be at a lower rate than the left one
	port4.anaWrite(110);
}

// hard left turn function, this function turns the vehicle left with a hard turn
void turnHardLeft() {
	// Setting the directions of the motors with the I/O expander to forward
	Motor_Directions(HIGH,LOW,HIGH,LOW);
	// setting the right motor's PWM to be at a higher rate than the left one
	port4.anaWrite(255);
	// setting the left motor's PWM to be at a lower rate than the right one
	port3.anaWrite(150);
}

// right turn function, this function turns the vehicle right with a slower turn
void turnRight() {
	// Setting the directions of the motors with the I/O expander to forward
	Motor_Directions(HIGH,LOW,HIGH,LOW);
	// setting the left motor's PWM to be at a higher rate than the right one
	port3.anaWrite(255);
	// setting the right motor's PWM to be at a lower rate than the left one
	port4.anaWrite(150);
}

// left turn function, this function turns the vehicle left with a slower turn
void turnLeft() {
  // Setting the directions of the motors with the I/O expander to forward
	Motor_Directions(HIGH,LOW,HIGH,LOW);
	// setting the right motor's PWM to be at a higher rate than the left one
	port4.anaWrite(255);
	// setting the left motor's PWM to be at a lower rate than the right one
	port3.anaWrite(190);
}

// stop function to stop the motors from turning
void stops() {
	// setting the motor directions to none, therefore stop the motors
	Motor_Directions(LOW,LOW,LOW,LOW);
}

// forward function, this function moves the vehicle forward
void forward() {
	// Setting the directions of the motors with the I/O expander to forward
	Motor_Directions(HIGH,LOW,HIGH,LOW);
	// setting both motor's PWM to be the same which means the vehicle will move forward
	// setting the right motor's PWM to 240
	port4.anaWrite(240);
	// setting the left motor's PWM to 240
	port3.anaWrite(240); 
  
}

// reverse function, this function moves the vehicle in reverse
void reverse() {
	// seeting the directions of the motors with the I/O expander to reverse
	Motor_Directions(LOW,HIGH,LOW,HIGH); 
	// setting both motor's PWM to be the same which means the vehicle will move backward
	// setting the right motor's PWM to 240
	port4.anaWrite(240);
	// setting the left motor's PWM to 240
	port3.anaWrite(240);   
}

/*	This function receives the status from the remote and the directions of the motors
*	Paramters; vehicle_status: sent by the remote: 1 means the remote will be controlling the vehicle
*													0 means the vehicle will be autonomous
*				direc: the directions of the motors, this will select forward or backward for each motor				
*				r_motor: the PWM rate of the right motor read from the remote
*				l_motor: the PWM rate of the left motor read from the remote							
*/
void jeenode_receive(bool *vehicle_status, unsigned char* direc, byte *r_motor,byte *l_motor, int *number)
{
	// setting a 3 byte array to hold the data received
	byte recv_msg[3];
	byte temp;
	// checking if data receiving is done  
	if (rf12_recvDone())
	{
		// placing the data received in the array
		memcpy(&recv_msg, (byte *)rf12_data, sizeof(byte)*3);  
		temp=recv_msg[0];
		// checking the vehicle status based on the MSNibble of the first byte
		if ((temp&0xF0) == 0xF0){
		   *vehicle_status = true;
		}else{
		   *vehicle_status = false;
                   return;
		}
		// setting the direction based on the LSNibble of the first byte
		*direc = recv_msg[0] & 0x0F;
		*direc = *direc << 4;
		
		// setting the right and left motor directions based on the second and third bytes
		*r_motor = recv_msg[1];
		*l_motor = recv_msg[2];
                *number=0;

        }else if(*number>15){
		// setting the vehicle status to false which means the vehicle is autonomous
		*vehicle_status = false; 
                *number=0;
	}
}

/*
*	This function is specific to the LED setup used for Group Hotel in ELEC3907 Project
*	distance is the status of the 4 LEDs used to show how far the vehicle is 
*   C1 controls the blue LED (Remote controlled) and C2 controls the red LED (not following)
*	distance is either 0x00 (00000000), 0x01 (00000001), 0x03 (00000011), 0x07 (00000111), 0x0F (00001111)
*/
void LEDs_control(unsigned char distance, bool C1, bool C2){      
	// setting temporary variables to be used
    unsigned char trigger=0;
	// temp is set to the MSb of the 4 LEDs
    unsigned char temp = distance&0x08;
	// temp is shifted 2 to the left (bit 5)
    temp = temp<<2;
	// bits 7 and 6 are set with the C2 and C1 values
	trigger |= C2<<7;
	trigger |= C1<<6;
	// bit 5 is set with the temp value
	trigger |= temp;
	// temp is set to the 3 MSb
    temp = distance&0x07;
	// temp is shifted 1 to the left (bits 3,2,1)
    temp = temp<<1;
	// bits 3,2,1 are set to temp value
    trigger |= temp;
	// setting the I/O expander port 1 to output the LEDs status
	MCP_WritePort(1, trigger);
}

// getting the IR values from the left and right IRs
// Returns a stucture of type IR_Vals
struct IR_Vals IR_Get()
{
	//port 1 is left, port 2 is right
	int IR_L=0,IR_R=0;
	// reading 20 IR values for both left and right
	for(int i=0; i<20; i++)
	{
		// reading the left IR
		IR_L += port1.anaRead(); // read from AIO of the “port”
		// reading the right IR
		IR_R += port2.anaRead(); // read from AIO of the “port"
	}
	  
	// averaging the IR values
	IR_L /= 20;
	IR_R /= 20;
	   
	// creating an IR structure
	struct IR_Vals IR_Values;
	// saving the values in the structure
	IR_Values.IR_L = IR_L;
	IR_Values.IR_R = IR_R;
	 
	// returning the structure
	return IR_Values;
}

// This function returns the ultrasound distance read
float getUltrasound()
{
	// duration variable
	long duration = 0;
	// distance variable
	float distance = 0;
	// reading 10 ultrasound values
	for(int n=0; n<10 ; n++){
		// setting port 2 to be output
		port2.mode(OUTPUT);
		// setting the output to low
		port2.digiWrite(LOW);
		// delaying the output by 2 microseconds
		delayMicroseconds(2);
		// setting the output to high
		port2.digiWrite( HIGH);
		// leaving the output high for 5 microseconds (5 microseconds pulse)
		delayMicroseconds(5);
		// turning the output back to low
		port2.digiWrite(LOW);
		// setting the port to input
		port2.mode(INPUT);
		// measuring the time the input is high
		duration = port2.pulse(HIGH); 
		// setting the distance based on the formula specified from the duration
		distance += (2*(duration/29));
	}
	// averaging the distance
	distance /= 10;
	// returning the distance
	return (distance);
}

// JeeNode setup function
void setup() {
	// initializing RF communications
	rf12_initialize(1, RF12_915MHZ, 69);//201,3 for proximity. 69,1 for joystick
	// starting I2C
	Start_I2C();
	// setting the directions of the I/O expander ports to output
	IOState CurrentState = MCP_SetDirection(0x00,0x00);
	// setting the analog ports to input in port2 1 and 2
	port1.mode2(INPUT); 
	port2.mode2(INPUT);
	// setting the digital ports to output in ports 3 and 4
	port3.mode(OUTPUT);
	port4.mode(OUTPUT);
}

  // vehicle status (remote controlled or autonomous)
  bool vehicle_status;
  // direction for the motors from the remote
  unsigned char direc;
  // left and right motor values from the remote
  byte r_motor;
  byte l_motor;
  int number=0;
  
// JeeNode loop function
void loop() {
  
  
  // receiving the data from the remote
  jeenode_receive(&vehicle_status, &direc, &r_motor, &l_motor, &number);
  number++;
  
  // setting the I/O expamder directions to be output
  IOState CurrentState = MCP_SetDirection(0x00,0x00);
  
  // checking if the vehicle is remote controlled or autonomous
  if (vehicle_status == true)
  {
	// vehicle is remote controlled
	// setting the LEDs to display blue for remote controlled
    LEDs_control(0x00, vehicle_status,0);
	// setting the motor directions from the remote
    MCP_WritePort(0, direc);
	// setting the right and left motor values from the remote
    port4.anaWrite(r_motor);
    port3.anaWrite(l_motor); 
  }
  else
  {   
	// temporary variables to hold the IR values and the distance from the ultrasound
    int IR_L=0,IR_R=0; 
    float duration;

	// reading the IR values structure and saving the values in the temporary variables earlier
    struct IR_Vals values = IR_Get();
    IR_L = values.IR_L;
    IR_R = values.IR_R;
	// getting the ultrasound distance
    duration = getUltrasound();
	// setting the LEDs depending on the status of the car
    if(duration > 100){
		// car is far away and not being autonomous
        LEDs_control(0x00,0,1);
    }else if(duration > 75){
		// car is far but following
        LEDs_control(0x01,0,0);
    }else if(duration > 45){
		// car is medium distance and following
        LEDs_control(0x03,0,0);
    }else if(duration > 25){
		// car is closer distance and following
        LEDs_control(0x07,0,0);
    }else if(duration > 15){
		// car is really close and following
        LEDs_control(0x0F,0,0);
    }else {
		// car is too close and not following
        LEDs_control(0x0F,0,1);
    } 

	// checking if the car is too close or too far to not follow 
    if((duration < 15) || (duration > 100)){
		// car is either too close or too far and cannot follow the car infront
		stops();
    }else{
		if( (IR_R-70) > IR_L ){
			// hard right turn must be performed
			turnHardRight();
		}else if( (IR_L-70) > IR_R ){
			// hard left turn must be performed
			turnHardLeft();
		}else if( IR_R > IR_L ){
			// normal right turn must be performed
			turnRight(); 
		}else{
			// normal left turn must be performed
			turnLeft();
		}
    }
   }
}

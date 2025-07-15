/*---------------------------------------------------------------------*/
/*         Module: alpha25.ino                                         */
/*         Author: DaWei                                               */
/*         Tak Tik 2025                                                */
/*         Acknowledgements: many thanks to Sivabalansm the goat       */
/*---------------------------------------------------------------------*/

#include <CrcLib.h>
#include "drive.h"
#include "button.h"
#include <Encoder.h>

#define LEFT_FRONT_WHEEL       CRC_PWM_1
#define LEFT_BACK_WHEEL   	   CRC_PWM_2
#define RIGHT_FRONT_WHEEL 	   CRC_PWM_3
#define RIGHT_BACK_WHEEL  	   CRC_PWM_4

#define BILDA_INTAKE	   	   CRC_PWM_5
#define BILDA_ROTATOR    	   CRC_PWM_7
#define BILDA_LIFT        	   CRC_PWM_8

#define BILDA_FLYWHEEL  	   CRC_PWM_9

#define VEX_LEFT_ARM 	       CRC_PWM_10
#define VEX_RIGHT_ARM   	   CRC_PWM_11
#define VEX_PUSH_FLYWHEEL	   CRC_PWM_12

#define ENCODER_LIFT_YELLOW	   CRC_ENCO_A
#define ENCODER_LIFT_GREEN	   CRC_ENCO_B

#define ENCODER_ROTATOR_YELLOW CRC_SERIAL_TXD1
#define ENCODER_ROTATOR_GREEN  CRC_SERIAL_RXD1

/* Safety Features Pre-boot */

// Safety feature: to stop all mouvement of controller devices of CrcDuino
char remoteDisconnectHalt[] = {CRC_PWM_1, CRC_PWM_2, CRC_PWM_3, CRC_PWM_4, CRC_PWM_5, CRC_PWM_7, CRC_PWM_8, CRC_PWM_9, CRC_PWM_10, CRC_PWM_11, CRC_PWM_12};


static void stopMouvement(char *pwmPins, char len) {
	for (char i = 0; i < len; i++) {
		CrcLib::SetPwmOutput(pwmPins[i], 0);
	};
};

/* Initialize PWM ouput */

CrcLib::Timer antijam;
void setup() {
    CrcLib::Initialize();

    // Initialize wheels
    CrcLib::InitializePwmOutput( LEFT_FRONT_WHEEL );
    CrcLib::InitializePwmOutput( LEFT_BACK_WHEEL  );
    CrcLib::InitializePwmOutput( RIGHT_FRONT_WHEEL );
    CrcLib::InitializePwmOutput( RIGHT_BACK_WHEEL );

    // Set wheel speed to 0 to stop *Rampage*
    CrcLib::SetPwmOutput( LEFT_FRONT_WHEEL , 0);
    CrcLib::SetPwmOutput( LEFT_BACK_WHEEL  , 0);
    CrcLib::SetPwmOutput( RIGHT_FRONT_WHEEL, 0);
    CrcLib::SetPwmOutput( RIGHT_BACK_WHEEL , 0);



    // Initialize Lift and Conveyors
    CrcLib::InitializePwmOutput( BILDA_INTAKE );
    CrcLib::InitializePwmOutput( BILDA_ROTATOR );
	CrcLib::InitializePwmOutput( BILDA_LIFT );

    // Set Lift and Conveyors speed to 0: prevents *Rampage*
    CrcLib::SetPwmOutput( BILDA_ROTATOR , 0);
    CrcLib::SetPwmOutput( BILDA_INTAKE , 0);
    CrcLib::SetPwmOutput( BILDA_LIFT, 0 );

	// Initialize Flywheels
	CrcLib::InitializePwmOutput( BILDA_FLYWHEEL);

	// Set flywheel speed to 0
	CrcLib::SetPwmOutput( BILDA_FLYWHEEL, 0);

    // Intialize Arms
    CrcLib::InitializePwmOutput( VEX_LEFT_ARM );
	CrcLib::InitializePwmOutput( VEX_RIGHT_ARM );

    // Set Arm speed to 0
    CrcLib::SetPwmOutput( VEX_LEFT_ARM, 0 );
	CrcLib::SetPwmOutput( VEX_RIGHT_ARM, 0 );

	//Initialize Encoder Pins for lift
	CrcLib::Initialize(ENCODER_LIFT_YELLOW);
	CrcLib::Initialize(ENCODER_LIFT_GREEN);
	
	//initialize Encoder Pins for Rotator
	CrcLib::Initialize(ENCODER_ROTATOR_YELLOW);
	CrcLib::Initialize(ENCODER_ROTATOR_GREEN);

	pinMode(ENCODER_ROTATOR_GREEN, INPUT);
	pinMode(ENCODER_ROTATOR_YELLOW, INPUT);

	//initialize push for flywheel
	CrcLib::InitializePwmOutput(VEX_PUSH_FLYWHEEL);
	CrcLib::SetPwmOutput(VEX_PUSH_FLYWHEEL , 0);


    // Intialize communications with computer
    // Serial.begin(9600);

};

// conveyor Toggle
Controller leftStick;
Controller rightStick;

char reverseDownMotor = 1;

static void switchUpMotors(char tf) {
	if (tf) {
		CrcLib::SetPwmOutput( BILDA_INTAKE,  reverseDownMotor * 128);
		
	} else {
		CrcLib::SetPwmOutput( BILDA_INTAKE, 0 );
		
	};

};



static void switchDownMotors(char tf) { //switch down motors is to reverse direction of intake
	if (tf) {

		reverseDownMotor = -1;
		speedLimitMouvement = 0.30;
		

	} else {
		reverseDownMotor = 1;
		speedLimitMouvement = 0.90;

	};

};

static void switchFlywheel(char tf) {
	if (tf) {
		presetHardValueF = 50;
	} else {
		presetHardValueF = 0;
	};
};

/* toggles buttons
downMotorsToggleButton      : toggles the intake system
upConveyorMotorToggleButton : toggles the top conveyor
flywheelToggleButton		: toggles the flywheel

*/

toggleButtonObj downMotorsToggleButton      = { .state = 0, .previousPush = 0 };
toggleButtonObj upConveyorMotorToggleButton = { .state = 0, .previousPush = 0 };
toggleButtonObj flywheelToggleButton 		= { .state = 0, .previousPush = 0 };

/* anti-spam buttons (only levaraging previousPush to not have repeated clicks)
computerSetSpamButton : click for the computer
startSpamButton       : start button stop spam for increase of speed
selectSpamButton      : select button stop spam for decrease of speed

*/
toggleButtonObj computerSetSpamButton = { .state = 0, .previousPush = 0 };
toggleButtonObj startSpamButton = { .state = 0, .previousPush = 0 };
toggleButtonObj selectSpamButton = { .state = 0, .previousPush = 0 };

// Button to presets for Daniel
char presetHardValueF = 0;

// Select and Start to adjust flywheels easily
char mini_adjust = 0;

// Total speed limit for the right joystick
double speedLimitMouvement;


//Encoder Setup

//Lift Encoder
Encoder liftEnc(ENCODER_LIFT_YELLOW, ENCODER_LIFT_GREEN);
short int oldPositionLift = -999;
liftEnc.write(0);

//Rotator Encoder
Encoder rotatorEnc(ENCODER_ROTATOR_YELLOW, ENCODER_ROTATOR_GREEN);
short int oldPositionRotator = -999;
rotatorEnc.write(0);

//PID settings
float kp = 1;
float kd = 0.025;
float ki = 0;
long PIDpreviousTime = 0;
newErrorLift = 0;
newErrorRotator = 0;
float eLiftIntegral = 0;
float eRotatorIntegral = 0;

//preset heights for lift
short int targetLift = 0;
short int lowestLift = 0;
short int level1Lift = 30;
short int level2Lift = 60;

//preset heights for rotator
short int targetRotator = 0;
short int horizontalRotator = 0;
short int verticalRotator = 75;

//max height limit for lift
short int maxLift = 90;

//lift delay to ensure smooth addition of the triggers (prevents instant jump in targetLift when the gachette is held down)
unsigned long previousMillisUp = 0; //stores last time targetLift was updated
unsigned long previousMillisDown = 0; //stores last time targetLift was updated
const long interval = 100; //interval to adjust the Lift values in milliseconds

void loop() {
    //Updates the CRCduino from the inputs from the controller
	CrcLib::Update();

    // Check if controller is connected
    if (CrcLib::IsCommValid()) { 

        // Get Analog values for the controllers
        leftStick.x  =  CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X);
        leftStick.y  = -CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y);

        rightStick.x =  CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X);
        rightStick.y = -CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_Y);


        calibrateStick(&leftStick);
	calibrateStick(&rightStick);
	

	/* toggle buttons for toggling intake system and conveyor */
	toggleButton(&downMotorsToggleButton,      CrcLib::ReadDigitalChannel(BUTTON::L1),           switchDownMotors);
	toggleButton(&upConveyorMotorToggleButton, CrcLib::ReadDigitalChannel(BUTTON::R1),           switchUpMotors);

	/* toggle button for toggling flywheel*/
	toggleButton(&flywheelToggleButton,		   CrcLib::ReadDigitalChannel(BUTTON::COLORS_LEFT),  switchFlywheel);
	/* Adjust speed of any presets by -1 or +1 using Select and Start. Center Logo button for reseting any changes */

	if (CrcLib::ReadDigitalChannel(BUTTON::LOGO)) {
		CrcLib::SetPwmOutput(VEX_PUSH_FLYWHEEL, 90);

	} else {
		CrcLib::SetPwmOutput(VEX_PUSH_FLYWHEEL, 0);
	}


	if (!startSpamButton.previousPush && CrcLib::ReadDigitalChannel(BUTTON::START)) {
		mini_adjust += 1;

	};

	startSpamButton.previousPush = CrcLib::ReadDigitalChannel(BUTTON::START);


	if (!selectSpamButton.previousPush && CrcLib::ReadDigitalChannel(BUTTON::SELECT)) {
		mini_adjust -= 1;

	};

	selectSpamButton.previousPush = CrcLib::ReadDigitalChannel(BUTTON::SELECT);

	/*--------------ENCODER FOR LIFT------------------*/
	
	//measuring current position of lift
	short int newPositionLift = liftEnc.read();
	if (newPositionLift != oldPositionLift) {
		oldPositionLift = newPositionLift;
	};
	//measuring current position of rotator
	short int newPositionRotator = rotatorEnc.read();
	if (newPositionRotator != oldPositionRotator) {
		oldPositionRotator = newPositionRotator;
	};

	//setting the target to the preset values
	if (CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN)) {
		targetLift = lowestLift;

	};

	if (CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT)) {
		targetLift = level1Lift;

	};

	if (CrcLib::ReadDigitalChannel(BUTTON::COLORS_UP)) {
		targetLift = level2Lift;

	};

	//manual adjustment of Lift target height using gachette L and gachette R
	unsigned long currentMillisUp = millis();
	short int moveLiftUp = (CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_R));
	if (moveLiftUp != 0 && (currentMillisUp - previousMillisUp >= interval)) {}
		targetLift = constrain(targetLift + 1, 0 , maxLift);
		previousMillisUp = currentMillisUp;

	};
	unsigned long currentMillisDown = millis();
	short int moveLiftDown = (CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_L));
	if (moveLiftDown != 0 && (currentMillisDown - previousMillisDown >= interval)) {
		targetLift = constrain(targetLift-1 , 0, maxLift);
		previousMillisDown = currentMillisDown;
	};

	//manual adjustment of Rotator target height using up and down on D pad
	if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_UP)) {
        targetRotator = horizontalRotator;
	}
	if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_DOWN)) {
    	targetRotator = verticalRotator;
	};	
	
	//PID setup values
	long PIDcurrentTime = micros();
	long et = PIDcurrentTime-PIDpreviousTime;
	float deltaTime = ((float)(PIDcurrentTime-PIDpreviousTime))/1.0e6;
	PIDpreviousTime = PIDcurrentTime;

	//PID controller for lift
	short int oldErrorLift = oldPositionLift - targetLift; //reverse signs if movement doesnt work
	float liftDeDt = (newErrorLift-oldErrorLift); //derivative
	newErrorLift = oldErrorLift;
	float eLiftIntegral = eLiftIntegral + (oldErrorLift * deltaTime); //integral

	short int signalLift = (short int)(kp * oldErrorLift + kd *liftDeDt + ki*eLiftIntegral); //resulting PID signal for lift

	//send PWM to the Bilda_Lift Motor
	short int powerLift = constrain(signalLift, -127, 127);
	Crc::SetPwmOutput(BILDA_LIFT, powerLift);

	//PID controller for rotator
	short int oldErrorRotator = oldPositionRotator - targetRotator; // reverse signs if movement doesnt work
	float rotatorDeDt = (newErrorRotator-oldErrorRotator); //derivative
	newErrorRotator = oldErrorRotator;
	float eRotatorIntegral = eRotatorIntegral + (oldErrorRotator * deltaTime); //integral
	
	short int signalRotator = (short int)(kp * oldErrorRotator + kd *rotatorDeDt + ki*eRotatorIntegral); //resulting PID signal for Rotator
	
	//send PWM to the Bilda_Rotator Motor
	short int powerRotator = constrain(signalRotator, -127, 127);
	Crc::SetPwmOutput(BILDA_ROTATOR, powerRotator);

	/*----------------------DEBUG CODE TO TUNE THE PID SYSTEM-------------------------*/
	// Serial.print(millis());
	// Serial.print(",");
	// Serial.print(targetLift);
	// Serial.print(",");
	// Serial.print(oldPositionLift);
	// Serial.print(",");
	// Serial.print(targetRotator);
	// Serial.print(",");
	// Serial.println(oldPositionRotator);
	// delay(50);
	

	//flywheel settings
	
	CrcLib::SetPwmOutput( BILDA_FLYWHEEL, constrain(0 - presetHardValueF - mini_adjust, -128, 127));
	  
	// Left and right arms using D-Pad 

	if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_LEFT)) {
           CrcLib::SetPwmOutput( VEX_LEFT_ARM, 96 );
		   CrcLib::SetPwmOutput( VEX_RIGHT_ARM, -96 );

	} else if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_RIGHT)) {
           CrcLib::SetPwmOutput( VEX_LEFT_ARM, -96 );
		   CrcLib::SetPwmOutput( VEX_RIGHT_ARM, 96 );

	} else {
           CrcLib::SetPwmOutput( VEX_LEFT_ARM, 0 );
		   CrcLib::SetPwmOutput( VEX_RIGHT_ARM, 0 );

	};
	
	if (leftStick.x != 0 || leftStick.y != 0 || rightStick.x!=0) {
		//Get Joystick Data
		short int linearX = leftStick.x;
		short int linearY = leftStick.y;
		short int rotationX = rightStick.x;

		//assigns value for each wheel
		short int frontRight = (linearY - rotationX - linearX);
		short int backRight = (linearY - rotationX + linearX);
		short int frontLeft = (linearY + rotationX + linearX);
		short int backLeft = (linearY + rotationX - linearX);

		//gets max value of each wheel
		short int maxWheel = max(abs(frontRight), abs(frontLeft), abs(backLeft), abs(backRight));

		CrcLib::SetPwmOutput( RIGHT_FRONT_WHEEL , (int) ((double) speedLimitMouvement * (double) -frontRight/maxWheel * 127));
	    CrcLib::SetPwmOutput( RIGHT_BACK_WHEEL  , (int) ((double) speedLimitMouvement * (double) -backRight/maxWheel * 127));
		CrcLib::SetPwmOutput( LEFT_FRONT_WHEEL  , (int) ((double) speedLimitMouvement * (double) frontLeft/maxWheel * 127));
	    CrcLib::SetPwmOutput( LEFT_BACK_WHEEL   , (int) ((double) speedLimitMouvement * (double) backLeft/maxWheel * 127));
	    
	} else {
		// Stop mouvement if joystick at coordiate (0, 0)

	    char wheelElements[] = { LEFT_FRONT_WHEEL, LEFT_BACK_WHEEL, RIGHT_FRONT_WHEEL, RIGHT_BACK_WHEEL };
	    stopMouvement(wheelElements, 4);
	};
	
	
    } else {

        // Stop ALL mouvement in case of a disconnect

	stopMouvement(remoteDisconnectHalt, 11);

	downMotorsToggleButton.state = 0;
	upConveyorMotorToggleButton.state  = 0;
	flywheelToggleButton.state = 0;
	
};



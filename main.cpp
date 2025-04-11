#include <windows.h>  // used to detect key presses

#include <ctime>     // used for loop timing
#include <iomanip>   // used to make the print outs look nicer
#include <iostream>  // used to print to the command line

//**************************//
//*** KEYBOARD SHORTCUTS ***//
//**************************//
// Hold P to pause
// press escape to exit the application

// m/s
#define gravity -9.806
// N
#define droneThrustForce 20
// Kg
#define droneMass 0.5
// Hz
#define PIDloopHz 100
// meters
#define targetAltitude 5

// PID Gains (try changing them and see how the output changes)

// these values reach the setpoint very fast
#define kP 2
#define kI 0.1
#define kD 0.5

// use these ones if you want to watch it reach the setpoint slower
// #define kP 0.25
// #define kI 0.05
// #define kD 0.05

// used to control the loop Hz
uint64_t prevTime = 0;

// struct to store physics data of the drone
struct objectData {
	double x;
	double y;
	double velX;
	double velY;
	double accelX;
	double accelY;
} drone;

// PID Logic
class PID {
   public:
	double Kp = 0;
	double Ki = 0;
	double Kd = 0;
	double iError = 0;
	double lastError = 0;

   public:
	// set all the values when an object is created
	PID(double Kp, double Ki, double Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	// update loop for the PID
	double update(double setPoint, double actualValue, double deltaTime) {
		// calculate the error
		double error = setPoint - actualValue;
		// intigrate the error
		iError += error * deltaTime;
		// calculate the output
		double output =
		    error * Kp + iError * Ki + (error - lastError) / deltaTime * Kd;
		// update the lastError variable for the next loop
		lastError = error;
		return (output);
	}
};

int main() {
	// create the PID object
	PID dronePid(kP, kI, kD);
	// do this until the escape key is pressed
	while (GetKeyState(27) >= 0) {
		// calculate the delta time
		double deltaTime = 1.0 / (clock() - prevTime);
		// store the current time for calculating the delta time
		prevTime = clock();

		// PID loop
		double throttle = dronePid.update(targetAltitude, drone.y, deltaTime);
		// keep the throttle in range
		if (throttle < 0) {
			throttle = 0;
		} else if (throttle > 1) {
			throttle = 1;
		}

		// drone physics calculations
		drone.accelY = (droneThrustForce * throttle + gravity) / droneMass;
		drone.velY += drone.accelY * deltaTime;
		drone.y += drone.velY * deltaTime;
		// keep the drone from going through the floor
		if (drone.y < 0) {
			drone.y = 0;
		}
		// print out the data
		std::cout << std::setprecision(4) << "Throttle " << throttle
		          << "\t Altitude " << drone.y << "\ttargetAltitude "
		          << targetAltitude << "\n";
		// pause the loop when P is pressed
		while (GetKeyState('P') < 0) {
			prevTime = clock();
		}
		Sleep(1);
	}
	return (0);
}
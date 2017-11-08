package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.Battery;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;

	NXTMotor leftMotor = null; // null objects are created to use all the
								// feautures of these objects without creating
								// them
	NXTMotor rightMotor = null; // again because they are already created in
								// guidance and perception

	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

	int leftMotorPower = 0;
	int rightMotorPower = 0;

	double velocity = 0.0;
	double angularVelocity = 0.0;

	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();

	ControlMode currentCTRLMODE = null;

	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	int lastTime = 0;

	double currentDistance = 0.0;
	double Distance = 0.0;

	// global Variables for PID
	double esum = 0; // static or final?????????????? esum has to be reserved in
					// storage
	double ealt = 0;

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * navigation object (to obtain the current position of the car from) and
	 * starts the control thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param navigation
	 *            corresponding main module Navigation class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 * @param leftMotor
	 *            corresponding NXTMotor object
	 * @param rightMotor
	 *            corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation,
			NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor) {
		this.perception = perception;
		this.navigation = navigation;
		this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.currentCTRLMODE = ControlMode.INACTIVE;

		this.encoderLeft = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		// WICHTIG eventuell †bergabe der kalibrierten Werte von Perception fŸr
		// Die PID regelung der Linienverfolgung?

		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * set velocity
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * 
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}

	/**
	 * set destination
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x,
	 *      double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}

	/**
	 * sets current pose
	 * 
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}

	/**
	 * set start time
	 */
	public void setStartTime(int startTime) {
		this.lastTime = startTime;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {

		switch (currentCTRLMODE) {
		case LINE_CTRL:
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			break;
		case VW_CTRL:
			update_VWCTRL_Parameter();
			exec_VWCTRL_ALGO();
			break;
		case SETPOSE:
			update_SETPOSE_Parameter();
			exec_SETPOSE_ALGO();
			break;
		case PARK_CTRL:
			update_PARKCTRL_Parameter();
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		this.drive(this.velocity, this.angularVelocity);
	}

	private void exec_SETPOSE_ALGO() {
		// Aufgabe 3.3
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		// Aufgabe 3.4
	}

	private void exec_INACTIVE() {
		this.stop();
	}

	/**
	 * DRIVING along black line Minimalbeispiel Linienverfolgung fuer gegebene
	 * Werte 0,1,2 white = 0, black = 2, grey = 1
	 */
//	 //Verbesserung des Minimalbeispieles mit den drei diskreten Werten
//	 private void exec_LINECTRL_ALGO() {
//	 leftMotor.forward();
//	 rightMotor.forward();
//	 int lowPower = 0;
//	 int highPower = 35;
//	 int medPower = 10;
//	
//	 // MONITOR (example)
//	 monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
//	 monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
//	
//	 if (this.lineSensorLeft == 2 && (this.lineSensorRight == 1)) {
//	
//	 // when left sensor is on the line, turn left
//	 leftMotor.setPower(lowPower);
//	 rightMotor.setPower(highPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn left");
//	
//	 } else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 1)) {
//	
//	 // when right sensor is on the line, turn right
//	 leftMotor.setPower(highPower);
//	 rightMotor.setPower(lowPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn right");
//	 } else if (this.lineSensorLeft == 2 && (this.lineSensorRight == 0)) {
//	
//	 // when left sensor is on the line, turn left
//	 leftMotor.setPower(lowPower);
//	 rightMotor.setPower(highPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn left");
//	
//	 } else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 0)) {
//	
//	 // when right sensor is on the line, turn right
//	 leftMotor.setPower(highPower);
//	 rightMotor.setPower(lowPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn right");
//	 } else if (this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
//	
//	 // when left sensor is on the line, turn left
//	 leftMotor.setPower(medPower); // nicht zu stark korrigieren..
//	 rightMotor.setPower(highPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn left");
//	
//	 } else if (this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
//	
//	 // when right sensor is on the line, turn right
//	 leftMotor.setPower(highPower);
//	 rightMotor.setPower(medPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("turn right");
//	 } else if (this.lineSensorLeft == 0 && this.lineSensorRight == 0) {
//	
//	 // when both sensors are next to the line, go straight
//	 leftMotor.setPower(highPower);
//	 rightMotor.setPower(highPower);
//	
//	 // MONITOR (example)
//	 monitor.writeControlComment("go straight");
//	 }
//	 }

	private void exec_LINECTRL_ALGO() {
		/**
		 * Idea: the robot is on its correct path, when it is positioned
		 * centrally on the line with both sensors returning their minimum value
		 * (calibrated value for white)
		 */
		leftMotor.forward();
		rightMotor.forward();

		this.lineSensorRight = perception.getRightLineSensorValue();
		this.lineSensorLeft = perception.getLeftLineSensorValue();

		/**
		 * if a sensor measures the calibrated value for e.g. white it returns
		 * 0; black(calibrated)-->100 the values are therefore largely
		 * uncorrelated to the actual brightness of the room but vary bright
		 * days still have a different fragmentation; e.g. the difference
		 * between 10 and 20 is greater when the brightness is high
		 */

		/**
		 * error values
		 */
		int errorRight = 0;
		int errorLeft = 0;
		int e = 0; // control error
		double delta = 0;// correcting factor
		/**
		 * correcting values
		 */
		double y = 0; // Stellwert

		int maxPower = 100;

		errorRight = this.lineSensorRight; // Asumption: the calibrated value
											// for white is 100 and every
											// deviation
		errorLeft = this.lineSensorLeft; // has to be corrected

		/**
		 * Definition for control deviation e: negative e -->too far left
		 * positive e -->too far right e==0 --> all right
		 */

		e = errorRight - errorLeft;
		if (e > 5 || e < -5)
			y = PID_control(e, 0.08, 0.02, 0.02, 1);
		else if (e > 90 || e < -90)			//starke kurve
			y = PID_control(e, 0.08, 0.02, 1, 1);
		else
			y = 0;
		/**
		 * if the control deviation is negative the correcting value will be
		 * negative aswell -->sign(y) ==-1 --> left turn -->sign(y) ==1 -->
		 * right turn
		 */
		if (y < 0) {
			y = -1 * y;
			if (y > maxPower/2)
				y = maxPower/2;
			leftMotor.setPower(maxPower/2 + (int) y);
//			if (e < -75) delta = 0;
//			else delta=0.5;
				rightMotor.setPower(maxPower/2 - (int) y); // speed of slower motor depending on the curvuture

		} else if (y > 0) {
			if (y > maxPower/2)
				y = maxPower/2;
			rightMotor.setPower(maxPower/2 + (int) y);
//			if (e > 75) delta = 0;
//			else delta=0.5;
			leftMotor.setPower(maxPower/2 -(int) (y));

		} else if (y == 0 || e == 0) {
			leftMotor.setPower(maxPower/2);
			rightMotor.setPower(maxPower/2);
		}
		if (esum>200 || esum < -200) 
			resetIntegralPID();

	}

	private double PID_control(double e, double KP, double KI, double KD,
			double Ta) {

		double y = 0; // Stellwert
		esum = esum + e;
		y = KP * e + KI * esum * Ta + KD * (e - ealt) / Ta;
		ealt = e;
		return y;
	}
	private void resetIntegralPID()
	{
		esum = 0;
	}

	private void stop() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot
	 * @param omega
	 *            angle velocity of the robot
	 */
	private void drive(double v, double omega) {
		// Aufgabe 3.2
	}
}
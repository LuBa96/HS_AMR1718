package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.geom.Line;
import lejos.nxt.Battery;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;
import lejos.nxt.Sound;

import java.util.ArrayList;

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

	double wheelD = 0; // wheeldiameter
	double width = 0; // width of track

	// global Variables for PID
	int esum = 0;
	double esumL = 0;
	double esumR = 0;
	int ealt = 0;
	double ealtL = 0;
	double ealtR = 0;
	int upperThreshold = 80;
	int maxPower = 0;
	int counter = 0;

	/**
	 * distinguish between straight driving and curving. PID optimized for
	 * straight driving is not suitable for curving and vice versa
	 */
	boolean boolTurn = false;
	boolean boolTurnL = false;
	boolean boolTurnR = false;

	/**
	 * variables for regWheelSpeed
	 */
	double leftAngleDiff = 0;
	double rightAngleDiff = 0;
	double leftDeltaTime = 0;
	double rightDeltaTime = 0;
	double omegaLeft = 0;
	double omegaRight = 0;
	double vLeft = 0;
	double vRight = 0;
	double errVLeft = 0;
	double errVRight = 0;
	double yL = 0;
	double yR = 0;
	double vLNew = 0;
	double vRNew = 0;
	int pwmLeft = 0;
	int pwmRight = 0;

	/**
	 * variables for controlled straight driving
	 */
	double startX = 0;
	double endX = 0;
	double startY = 0;
	double endY = 0;
	/**
	 * transformed variables
	 */
	double xRotKOS = 0;
	double yRotKOS = 0;
	double phiRotKOS = 0;
	boolean pathEnd = false;
	CoordinateSystem CoSys = null;
	Pose jetzt = null;

	Line guideLine = null;

	/**
	 * Variables for PID
	 */
	double errYSum = 0;
	double errYAlt = 0;
	double lastError = 0;
	double lastLastError = 0;

	/**
	 * demo program
	 */
	boolean demo1 = false;
	boolean demo2 = false;
	boolean demo3 = false;
	boolean demo4 = false;
	boolean demo5 = false;
	boolean demoFin = false;

	/**
	 * variables for methods rotateXDeg() and driveXCm()
	 */
	boolean boolRotate = false;
	boolean boolDrive = false;
	int angleDeg = 0;
	int startAngleDeg = 0;

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
		this.lineSensorRight = perception.getRightLineSensorValue();
		this.lineSensorLeft = perception.getLeftLineSensorValue();

		wheelD = 5.6; // wheeldiameter
		width = 13; // width of track

		// global Variables for PID
		esum = 0;
		esumL = 0;
		esumR = 0;
		ealt = 0;
		ealtL = 0;
		ealtR = 0;
		upperThreshold = 70;
		maxPower = 90;

		/**
		 * distinguish between straight driving and curving. PID optimized for
		 * straight driving is not suitable for curving and vice versa
		 */
		boolTurn = false;
		boolTurnL = false;
		boolTurnR = false;

		/**
		 * demo program
		 */
		demo1 = true;
		demo2 = false;
		demo3 = false;
		demo4 = false;
		demo5 = false;
		startAngleDeg = 0;
		startX = 0;
		startY = 0;

		CoSys = new CoordinateSystem();
		jetzt = new Pose();

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
		this.currentPosition.setHeading((float) (currentPosition.getHeading()
				/ Math.PI * 180));
		this.currentPosition.setLocation(currentPosition.getX() * 100,
				currentPosition.getY() * 100);

	}

	public void updateStartPose() {
		this.startPosition = navigation.getPose();
		startAngleDeg = (int) (this.startPosition.getHeading() / Math.PI * 180);
		startX = this.startPosition.getX() * 100;
		startY = this.startPosition.getY() * 100;

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

	public boolean getDemoStatus() {
		return demoFin;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {
		switch (currentCTRLMODE) {
		case DEMO1_CTRL:
			update_SETPOSE_Parameter();
			Control_Demo_1();
			break;
		case DEMO2_CTRL:
			update_SETPOSE_Parameter();
			Control_Demo_2();
			break;
		case DEMO3_CTRL:
			update_SETPOSE_Parameter();
			Control_Demo_3();
			break;
		case LINE_CTRL:
			update_SETPOSE_Parameter();
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			break;
		case LEFT_CRV_CTRL:
			update_SETPOSE_Parameter();
			exec_driveCurve90();
			break;
		case RIGHT_CRV_CTRL:
			update_SETPOSE_Parameter();
			exec_driveCurve90();
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

	/**
	 * public methods for direction to check for transition into curveMode
	 */
	public boolean getCurveMode() {
		return boolTurn;
	}

	public boolean getLeftTurn() {
		return boolTurnL;
	}

	public boolean getRightTurn() {
		return boolTurnR;
	}

	public double getXstrich() {
		return xRotKOS;
	}

	public double getYstrich() {
		return yRotKOS;
	}

	public double getesum() {
		return esum;
	}

	/**
	 * Method to reset the accumulated error in esum to zero; prevents
	 * "overregulation" by the I-Factor
	 */
	public void resetIntegralPID() {
		esum = 0;
	}

	/**
	 * reset accumulated error for regWheelDrive use for change between straight
	 * drive and turn
	 */
	public void resetIntegralRWD() {
		esumL = 0;
		esumR = 0;
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
		this.lineSensorRight = perception.getRightLineSensorValue();
		this.lineSensorLeft = perception.getLeftLineSensorValue();
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in degree
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		this.drive(this.velocity, this.angularVelocity);
	}

	/**
	 * controlled straight driving
	 * 
	 * theoretical preparations showed stable behavior for PD and PID
	 * Transfer-Function of deviation e¡(2) = v_0 * omega --> double integrator
	 * 
	 * transform koordinates --> turn x_2 in drive direction when robot is at
	 * the start position of its course
	 * 
	 * !GUIDANCE has to update angle, start and end points before starting!
	 *
	 */
	private void exec_SETPOSE_ALGO() {
		this.followPath(this.velocity);
	}

	/**
	 * follow a straight line in respect to the start pose (heading)
	 * 
	 * @param vo
	 */
	private void followPath(double vo) {
		double y;

		/**
		 * set start pose as origin for coordinate system and follow the path
		 */
		if (CoSys.getPointOfOrigin() == null || pathEnd) {
			jetzt.setLocation(this.currentPosition.getX(),
					this.currentPosition.getY());
			jetzt.setHeading(this.currentPosition.getHeading());
			Sound.systemSound(true, 0);
			CoSys.setPointOfOrigin(jetzt);
			pathEnd = false;
			errYAlt = 0;
			errYSum = 0;
		} else {
			// do nothing
		}
		phiRotKOS = CoSys.getTransformedHeading(this.currentPosition);
		xRotKOS = CoSys.getTransformedPose(this.currentPosition).getX();
		yRotKOS = CoSys.getTransformedPose(this.currentPosition).getY();
			

		/*
		 * assumption: follow x1-axis --> x2=0
		 */
		errYAlt = yRotKOS - errYAlt;

		y = 0.65 * yRotKOS + 0.00 * errYSum + 10 * lastError;
		drive(vo, -y);

		errYSum = errYSum + yRotKOS;
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
	 * this method detects a right or left turn by checking if a typical
	 * pattern: both white (grey)-->black and white-->both bright white is
	 * detected
	 */
	private void detectTurn(double eLeft, double eRight) {

		if ((eLeft <= 25) || (eRight <= 25)) {
			if ((eLeft <= 25) && (counter > 0)) {
				counter = counter - 1;
			} else if ((eLeft <= 25) && (counter == 0)) {
				boolTurnL = true;
				boolTurn = true;
				counter = 2;
			}
			if ((eRight <= 25) && (counter > 0)) {
				counter = counter - 1;
			} else if ((eRight <= 25) && (counter == 0)) {
				boolTurnR = true;
				boolTurn = true;
				counter = 2;
			}
		} else {
			boolTurnR = false;
			boolTurnL = false;
			boolTurn = false;

		}

	}

	private void exec_LINECTRL_ALGO() {
		/**
		 * if robot gets close to a curve--> decelerate TODO smoother!!!
		 */

		maxPower = 60;

		leftMotor.forward();
		rightMotor.forward();

		this.lineSensorRight = perception.getRightLineSensorValue();
		this.lineSensorLeft = perception.getLeftLineSensorValue();

		/**
		 * if a sensor measures the calibrated value for e.g. black it returns
		 * 0; white(calibrated)-->100 the values are therefore largely
		 * uncorrelated to the actual brightness of the room but very bright
		 * days still have a different fragmentation; e.g. the difference
		 * between 10 and 20 is greater when the brightness is high
		 */

		/**
		 * error values
		 */
		int errorRight = 0;
		int errorLeft = 0;
		int e = 0; // control error

		/**
		 * correcting values
		 */
		double y = 0; // Stellwert

		/**
		 * maximum value for white is 100 and minimum value for black is 0
		 * higher and lower values have been returned if the ground is
		 * "more white" or "more black" than calibrated
		 */
		// correct left value
		if (this.lineSensorRight > 100)
			errorRight = 100;
		else if (this.lineSensorRight < 0)
			errorRight = 0;
		else
			errorRight = this.lineSensorRight;
		// correct right value
		if (this.lineSensorLeft > 100)
			errorLeft = 100;
		else if (this.lineSensorLeft < 0)
			errorLeft = 0;
		else
			errorLeft = this.lineSensorLeft;

		/**
		 * Definition for control deviation e: negative e --> too far left;
		 * positive e --> too far right; e==0 --> no error
		 */
		e = errorRight - errorLeft;
		detectTurn(errorLeft, errorRight);

		/**
		 * regular straight driving mode KP = 0.1 KI = 0.002 KD = 0.06
		 */
		y = PID_control(e, 0.1, 0.002, 0.06, 1);

		/**
		 * correction to the right side
		 */
		if (y < 0) {
			if (y < -(maxPower / 2))
				y = -(maxPower / 2);
			regWheelSpeed(((int) (maxPower / 2) - (int) y) / 2,
					((int) (maxPower / 2) + (int) y) / 2);
			// leftMotor.setPower((int) (maxPower / 2) - (int) y);
			// rightMotor.setPower((int) (maxPower / 2) + (int) y);
		}
		/**
		 * correction to the left side
		 */
		else if (y > 0) {
			if (y > maxPower / 2)
				y = maxPower / 2;
			regWheelSpeed(((int) (maxPower / 2) - (int) y) / 2,
					((int) (maxPower / 2) + (int) y) / 2);
			// rightMotor.setPower((int) (maxPower / 2) + (int) y);
			// leftMotor.setPower((int) (maxPower / 2) - (int) y);
		}
		/**
		 * Drive straight
		 */
		else if (y == 0) {
			regWheelSpeed(((int) (maxPower / 2)) / 2,
					((int) (maxPower / 2)) / 2);
			// leftMotor.setPower((int) maxPower / 2);
			// .setPower((int) maxPower / 2);
		}

		/**
		 * Anti-Reset-Windup
		 */
		if ((esum > 10000) || (esum < -10000))
			resetIntegralPID();

	}

	/**
	 * robot turns 90 degrees for curve mode
	 * 
	 * @param direction
	 *            > 0 right curve
	 */
	private void exec_driveCurve90() {
		double xMomentary = this.currentPosition.getX();
		double yMomentary = this.currentPosition.getY();
		angleDeg = (int) (this.currentPosition.getHeading());
		double disMomentary = Math.sqrt(Math.pow((xMomentary - startX), 2)
				+ Math.pow((yMomentary - startY), 2));
		if (disMomentary <= 4)
			drive(5, 0);
		else {
			switch (currentCTRLMODE) {
			case LEFT_CRV_CTRL:
				if ((angleDeg - startAngleDeg) <= 80) {
					drive(0, 40);
					// Sound.buzz();
				} else {
					boolTurn = false;
					boolTurnL = false;
					rightMotor.stop();
					leftMotor.stop();
				}
				break;
			// right curve
			case RIGHT_CRV_CTRL:
				if ((angleDeg - startAngleDeg) >= -80) {
					drive(0, -40);
					// Sound.buzz();
				} else {
					boolTurn = false;
					boolTurnR = false;
					rightMotor.stop();
					leftMotor.stop();
				}
				break;
			default:
				rightMotor.stop();
				leftMotor.stop();
				break;
			}
		}
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot
	 * @param omega
	 *            angle velocity of the robot left turn is mathematical positive
	 *            right turn is mathematical negative
	 */
	private void drive(double v, double omega) {
		// Aufgabe 3.2
		leftMotor.forward();
		rightMotor.forward();
		double radiusM = 0; // radius of rotation; r=0--> rotate without
							// translation; r->inf -->straight driving

		double speed = 0;

		/**
		 * If omega is zero, don't rotate; translate with both wheels turning
		 * with v
		 */
		if (omega == 0) {
			vLeft = v; // needs to be improved with regulated straight
						// driving
			vRight = v;
		}

		/**
		 * If v is zero, don't translate; rotate left if omega is positive and
		 * right if omega is negative
		 */
		else if (v == 0) {
			// middle of the robot = center of rotation
			speed = omega * (Math.PI / 180) * width / 2;
			vLeft = -speed;
			vRight = speed;
		}
		/**
		 * this works for both orientations, because radiusM and omega are
		 * negative on a right turn and therefore the sign of width/2 changes
		 * according to the orientation of the rotation
		 */
		else {
			omega = omega * (Math.PI / 180);
			radiusM = v / omega;
			speed = (radiusM - width / 2) * omega;
			vLeft = speed;
			speed = (radiusM + width / 2) * omega;
			vRight = speed;

		}

		/**
		 * calculate PWM values and set speed for each wheel solution WITH
		 * regulated speed control
		 */
		regWheelSpeed(vLeft, vRight);

	}

	private int getPWM(double v) {
		int pwm = 0;
		pwm = (int) (3.9 * v - 1) / 2;

		return pwm;
	}

	/**
	 * rotation speed regulation
	 */
	private void regWheelSpeed(double vL, double vR) {
		/**
		 * get angledifference in degree and time in msec
		 */
		this.angleMeasurementLeft = this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = this.encoderRight.getEncoderMeasurement();

		leftAngleDiff = this.angleMeasurementLeft.getAngleSum();
		leftAngleDiff = leftAngleDiff * Math.PI / 180;

		leftDeltaTime = this.angleMeasurementLeft.getDeltaT();
		leftDeltaTime = leftDeltaTime / 1000;

		rightAngleDiff = this.angleMeasurementRight.getAngleSum();
		rightAngleDiff = rightAngleDiff * Math.PI / 180;

		rightDeltaTime = this.angleMeasurementRight.getDeltaT();
		rightDeltaTime = rightDeltaTime / 1000;

		/**
		 * idea: get the average speed of the individual wheels with deltaPhi
		 * and deltaT
		 */
		omegaLeft = leftAngleDiff / leftDeltaTime;
		omegaRight = rightAngleDiff / rightDeltaTime;

		/**
		 * v = w * r; actual value omega is the rotation of the individual
		 * wheels, not the rotation of the robot
		 */
		vLeft = omegaLeft * wheelD / 2;
		vRight = omegaRight * wheelD / 2;

		// LCD.drawString("vL: " + vLeft, 0, 6);
		// LCD.drawString("vR: " + vRight, 0, 7);
		/**
		 * Definition of control error; individual regulation of each wheel; vL
		 * and vR are the desired values
		 */
		errVLeft = vL - vLeft;
		errVRight = vR - vRight;

		/**
		 * calculate control values PID
		 */
		esumL = esumL + errVLeft;
		esumR = esumR + errVRight;

		yL = 0.5 * errVLeft + 0.15 * esumL + 0 * (errVLeft - ealtL);
		yR = 0.5 * errVRight + 0.15 * esumR + 0 * (errVRight - ealtR);

		ealtL = errVLeft;
		ealtR = errVRight;

		vLNew = vL + yL;
		vRNew = vR + yR;

		/**
		 * calculate PWM values and set speed
		 */
		pwmLeft = 0;
		pwmRight = 0;
		pwmLeft = getPWM(vLNew);
		pwmRight = getPWM(vRNew);
		leftMotor.setPower(pwmLeft);
		rightMotor.setPower(pwmRight);
		/**
		 * Anti-Reset-Windup
		 */
		if ((esumL > 1500) || (esumL < -1500))
			esumL = 0;

		if ((esumR > 1500) || (esumR < -1500))
			esumR = 0;

	}

	/**
	 * Method to calculate the control value for a specific control error esum
	 * and ealt are global variables with the accumulated errors (I) and last
	 * error (D) respectively
	 * 
	 * @param e
	 *            control error
	 * @param KP
	 *            proportional factor
	 * @param KI
	 *            integral factor
	 * @param KD
	 *            differential factor
	 * @param Ta
	 *            sample time
	 * @return y control value
	 */
	private double PID_control(int e, double KP, double KI, double KD, double Ta) {

		double y = 0;
		esum = esum + e;
		y = KP * e + KI * esum * Ta + KD * (e - ealt) / Ta;
		ealt = e;
		// LCD.drawString("es: " + esum + " ea: " + ealt, 0, 8);
		return y;
	}

	private void stop() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

	/**
	 * turn X degrees with given rotatory speed
	 * 
	 * @param omega
	 */
	private boolean rotateXDeg(double omega, double angle) {
		boolRotate = false;
		drive(0, omega);
		if ((int) (this.currentPosition.getHeading()) - startAngleDeg >= angle) {
			updateStartPose();
			leftMotor.stop();
			rightMotor.stop();
			// reset esumL and esumR, otherwise the accumulated error for
			// straight driving would be used for turning
			esumL = 0;
			esumR = 0;
			boolRotate = true;
		}
		return boolRotate;
	}

	public Pose origin() {
		return CoSys.getPointOfOrigin();
	}

	public double Xstrich() {
		return xRotKOS;
	}

	public double Ystrich() {
		return yRotKOS;
	}

	/**
	 * drive X cm with given translatory velocity
	 * 
	 * @param vo
	 * @param dis
	 * @return
	 */
	private boolean driveXCm(double vo, double dis) {
		/*
		 * navigation (currentPosition) returns value in cm
		 */
		double disMomentary = 0;
		boolDrive = false;
		followPath(vo);
		if (CoSys.getPointOfOrigin() == null)
			disMomentary = 0;
		else {
			/*
			 * CoSys.getPointofOrigin.getX() returns value in cm
			 */
			disMomentary = CoSys.getTransformedPoint(this.currentPosition)
					.distance(0, 0);
		}
		if (disMomentary >= dis) {
			boolDrive = true;
			leftMotor.stop();
			rightMotor.stop();
		}
		return boolDrive;
	}

	/**
	 * Method for first presentation; follow a given routine and continue in
	 * Line Control Mode at last; sound a beep sequence after each finished step
	 */
	private void Control_Demo_1() {
		if (demo5) {
			counter = 50;
			demo1 = true;
			demo5 = false;
			demoFin = true;

		} else
			Control_Demo();

	}

	/**
	 * Method for second presentation; follow a given routine and continue in
	 * Line Control Mode at last; sound a beep sequence after each finished step
	 */
	private void Control_Demo_2() {
		if ((demo5 && (Math.abs((float) (navigation.getPose().getX())) <= 5) && (Math
				.abs((float) (navigation.getPose().getY())) <= 5))) {
			updateStartPose();
			if (rotateXDeg(30, 180))
				demoFin = true;

		} else if (demo5) {
			// TODO Guidance has to put control into line control mode
		} else
			Control_Demo();

	}

	private void Control_Demo_3() {

	}

	private void Control_Demo() {
		double x = this.currentPosition.getX();
		double y = this.currentPosition.getY();
		double dis = Math.sqrt(x * x + y * y);
		/**
		 * 120 cm with 10 cm/s straight driving
		 */
		if (demo1) {
			if (!driveXCm(10,120)) {
				// drive
			} else {
				Sound.systemSound(true, 0);
				demo2 = true;
				demo1 = false;
				// reset esumL and esumR, otherwise the accumulated error for
				// straight driving would be used for turning
				esumL = 0;
				esumR = 0;
				pathEnd = true;
				updateStartPose();
			}
			// if (dis >= 120) {
			// Sound.systemSound(true, 0);
			// demo2 = true;
			// demo1 = false;
			// updateStartPose();
			// leftMotor.stop();
			// rightMotor.stop();
			// // reset esumL and esumR, otherwise the accumulated error for
			// // straight driving would be used for turning
			// esumL = 0;
			// esumR = 0;
			// }
		}

		/**
		 * 90 deg turn with 15deg/sec
		 */
		else if (demo2) {
			// drive(0, 15);
			if (!rotateXDeg(15, 90)) {
				// rotate
			} else {
				Sound.systemSound(true, 1);
				demo3 = true;
				demo2 = false;
				// reset esumL and esumR, otherwise the accumulated error for
				// straight driving would be used for turning
				esumL = 0;
				esumR = 0;
				pathEnd = true;
				updateStartPose();
			}
		}

		/**
		 * 30 cm with 5 cm/s straight driving
		 */
		else if (demo3) {
			if (!driveXCm(5, 30)) {
				// drive
			} else {
				Sound.systemSound(true, 0);
				Sound.systemSound(true, 1);
				demo4 = true;
				demo3 = false;
				// reset esumL and esumR, otherwise the accumulated error for
				// straight driving would be used for turning
				esumL = 0;
				esumR = 0;
				pathEnd = true;
				updateStartPose();
			}
			// drive(5, 0);
			// // driveStraight(5, 0);
			// double disMomentary = Math.sqrt(Math.pow((x - startX), 2)
			// + Math.pow((y - startY), 2));
			// if (disMomentary >= 33) {
			// Sound.systemSound(true, 0);
			// Sound.systemSound(true, 1);
			// demo4 = true;
			// demo3 = false;
			// updateStartPose();
			// leftMotor.stop();
			// rightMotor.stop();
			// // reset esumL and esumR, otherwise the accumulated error for
			// // straight driving would be used for turning
			// esumL = 0;
			// esumR = 0;
			// }
		}

		/**
		 * -90 deg turn with 30deg/sec
		 */
		else if (demo4) {
			drive(0, -30);
			LCD.drawString("c: " + (this.currentPosition.getHeading()), 0, 6);
			LCD.drawString("s: " + startAngleDeg, 0, 7);
			if ((int) (this.currentPosition.getHeading()) - startAngleDeg <= -80) {
				Sound.systemSound(true, 1);
				Sound.systemSound(true, 1);
				demo5 = true;
				counter = 8;
				demo4 = false;
				leftMotor.stop();
				rightMotor.stop();
				esumL = 0;
				esumR = 0;
			}
		}

	}
}
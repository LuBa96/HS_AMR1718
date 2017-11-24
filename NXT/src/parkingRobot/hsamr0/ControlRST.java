package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.Battery;
import lejos.nxt.LCD;
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

	double wheelD = 5.6; // wheeldiameter
	double width = 13; // width of track

	// global Variables for PID
	double esum = 0;
	double esumL = 0;
	double esumR = 0;
	double ealt = 0;
	double ealtL = 0;
	double ealtR = 0;
	int upperThreshold = 65;
	int grey = 35;
	int lowerThreshold = 20;
	int maxPower = 70;
	int counter = 0;
	/**
	 * distinguish between straight driving and curving. PID optimized for
	 * straight driving is not suitable for curving and vice versa
	 */
	boolean curve;
	boolean curveL;
	boolean curveR;
	/**
	 * demo program
	 */
	boolean demo1 = true;
	boolean demo2;
	boolean demo3;
	boolean demo4;
	boolean demo5;
	int startAngle = 0;
	double startDis = 0;

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
		//Control_Demo_1();
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
		this.lineSensorRight = perception.getRightLineSensorValue();
		this.lineSensorLeft = perception.getLeftLineSensorValue();
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
	/*
	 * // //Verbesserung des Minimalbeispieles mit den drei diskreten Werten //
	 * private void exec_LINECTRL_ALGO() { // leftMotor.forward(); //
	 * rightMotor.forward(); // int lowPower = 0; // int highPower = 35; // int
	 * medPower = 10; // // // MONITOR (example) //
	 * monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft); //
	 * monitor.writeControlVar("RightSensor", "" + this.lineSensorRight); // //
	 * if (this.lineSensorLeft == 2 && (this.lineSensorRight == 1)) { // // //
	 * when left sensor is on the line, turn left //
	 * leftMotor.setPower(lowPower); // rightMotor.setPower(highPower); // // //
	 * MONITOR (example) // monitor.writeControlComment("turn left"); // // }
	 * else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 1)) { // //
	 * // when right sensor is on the line, turn right //
	 * leftMotor.setPower(highPower); // rightMotor.setPower(lowPower); // // //
	 * MONITOR (example) // monitor.writeControlComment("turn right"); // } else
	 * if (this.lineSensorLeft == 2 && (this.lineSensorRight == 0)) { // // //
	 * when left sensor is on the line, turn left //
	 * leftMotor.setPower(lowPower); // rightMotor.setPower(highPower); // // //
	 * MONITOR (example) // monitor.writeControlComment("turn left"); // // }
	 * else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 0)) { // //
	 * // when right sensor is on the line, turn right //
	 * leftMotor.setPower(highPower); // rightMotor.setPower(lowPower); // // //
	 * MONITOR (example) // monitor.writeControlComment("turn right"); // } else
	 * if (this.lineSensorLeft == 1 && this.lineSensorRight == 0) { // // //
	 * when left sensor is on the line, turn left //
	 * leftMotor.setPower(medPower); // nicht zu stark korrigieren.. //
	 * rightMotor.setPower(highPower); // // // MONITOR (example) //
	 * monitor.writeControlComment("turn left"); // // } else if
	 * (this.lineSensorRight == 1 && this.lineSensorLeft == 0) { // // // when
	 * right sensor is on the line, turn right // leftMotor.setPower(highPower);
	 * // rightMotor.setPower(medPower); // // // MONITOR (example) //
	 * monitor.writeControlComment("turn right"); // } else if
	 * (this.lineSensorLeft == 0 && this.lineSensorRight == 0) { // // // when
	 * both sensors are next to the line, go straight //
	 * leftMotor.setPower(highPower); // rightMotor.setPower(highPower); // //
	 * // MONITOR (example) // monitor.writeControlComment("go straight"); // }
	 * // }
	 */

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
		/**
		 * correcting values
		 */
		double y = 0; // Stellwert
		/**
		 * Threshold and maxPower
		 */

		/*
		 * //!!!alternative Linecontrol!!! errorRight = this.lineSensorRight;
		 * errorLeft = this.lineSensorLeft; int maxPower = 70; int soll= 50;
		 * //genau auf der Grenze e = errorLeft/2 - soll; if(!((errorRight < 1)
		 * || (errorLeft > 99)) && !((errorLeft < 1) || (errorRight > 99))) {
		 * y=PID_control(e, 0.15, 0.02, 0.02, 1); if (y < 0) { y = -1 * y; if (y
		 * > maxPower / 2) y = maxPower / 2; leftMotor.setPower(maxPower / 2 +
		 * (int) y); rightMotor.setPower(maxPower / 2 - (int) y); // speed of
		 * slower // motor // depending on // the curve
		 * 
		 * /** correction to the left side
		 * 
		 * } else if (y > 0) { if (y > maxPower / 2) y = maxPower / 2;
		 * rightMotor.setPower(maxPower / 2 + (int) y);
		 * leftMotor.setPower(maxPower / 2 - (int) y);
		 * 
		 * /** Drive straight
		 * 
		 * } else if (y == 0) { leftMotor.setPower(maxPower / 2);
		 * rightMotor.setPower(maxPower / 2); } } else if (errorRight < 1 ||
		 * (errorLeft > 99)) { //Kurve rechts leftMotor.setPower((int) (maxPower
		 * * 2 / 3)); rightMotor.setPower((int) (-maxPower * 2 / 3)); } else if
		 * ((errorLeft < 1) || (errorRight > 99)) { //Kurve links
		 * rightMotor.setPower((int) (maxPower * 2 / 3));
		 * leftMotor.setPower((int) (-maxPower * 2 / 3)); }
		 * 
		 * //!!!alternative Linecontrol!!!
		 */

		if (this.lineSensorRight > 100)
			errorRight = 100;
		else if (this.lineSensorRight < 0)
			errorRight = 0;
		else
			errorRight = this.lineSensorRight; // Asumption: the calibrated
												// value
		// for white is 100 and every
		// deviation has to be corrected
		if (this.lineSensorLeft > 100)
			errorLeft = 100;
		else if (this.lineSensorLeft < 0)
			errorLeft = 0;
		else
			errorLeft = this.lineSensorLeft;

		/**
		 * Definition for control deviation e: negative e -->too far left
		 * positive e -->too far right e==0 --> all right
		 */
		e = errorRight - errorLeft;

		/**
		 * as long as e is greater than the lower threshold for exiting the
		 * curve mode continue in curve mode
		 */
		if (curve && ((e < lowerThreshold) && (e > -lowerThreshold))) // &&
																		// ((errorRight
																		// >
																		// grey)
																		// &&
																		// (errorLeft
																		// >
																		// grey))
		// end of strongcurve
		// compare errorRight and errorLeft to make sure that the sensors
		// are not seeing black and black
		{
			/**
			 * State Transition: curve --> straight
			 */
			if ((errorRight > grey) && (errorLeft > grey)) {
				curve = false;
				curveL = false;
				curveR = false;
			}
			/**
			 * remain in curve mode
			 */
			else
				curve = true;
		}

		if ((!(curve)) && ((e > upperThreshold) || (e < -upperThreshold))) {
			// dont change into curve mode too frequently
			/**
			 * remain in straight mode until counter is 0
			 */
			if (counter > 0) {
				counter = counter - 1;
				y = PID_control(e, 0.15, 0.02, 0.02, 1);
				/**
				 * State Transition: straight --> curve
				 */
			} else {
				curve = true;
				counter = 3; // about 0.3 seconds of latency with a sleep of
								// 100ms
			}
		}
		// curve
		if (curve)
			driveCurve(e);
		// straight
		if (!curve) {
			/**
			 * no sign of a curve; normal routine for straight driving
			 */
			if ((errorRight > grey) && (errorLeft > grey)) // both sensors
															// show
															// more
															// or less white
			{
				y = PID_control(e, 0.15, 0.02, 0.02, 1);

				/**
				 * correction to the right side
				 */
				if (y < 0) {
					y = -1 * y;
					if (y > maxPower / 2)
						y = maxPower / 2;
					leftMotor.setPower((int) (maxPower / 2) + (int) y);
					rightMotor.setPower((int) (maxPower / 2) - (int) y); // speed
																			// of
					// slower
					// motor
					// depending
					// on
					// the curve

					/**
					 * correction to the left side
					 */
				} else if (y > 0) {
					if (y > maxPower / 2)
						y = maxPower / 2;
					rightMotor.setPower((int) (maxPower / 2) + (int) y);
					leftMotor.setPower((int) (maxPower / 2) - (int) y);

					/**
					 * Drive straight
					 */
				} else if (y == 0) {
					leftMotor.setPower((int) maxPower / 2);
					rightMotor.setPower((int) maxPower / 2);
				}
			}
			// if both sensors show black, continue curve
			else if ((errorRight < grey) && (errorLeft < grey)) {
				curve = true;
				driveCurve(ealt);
			}
		}
		/**
		 * Anti-Reset-Windup
		 */
		if ((esum > 100) || (esum < -100))
			resetIntegralPID();

	}

	private void driveCurve(double e) {
		double y;
		/**
		 * left curve
		 */
		if ((e > 0) || curveL) {
			/**
			 * dont change curves directly after one another
			 */
			if (curveR) {
				// if the robot has been in a right curve in the last cycle it
				// may not change into left curve right
				// until the next cycle finishes
				curveR = false;
				y = PID_control(e, 0.15, 0.02, 0.02, 1);
				rightMotor.setPower((int) (maxPower / 2) + (int) y);
				leftMotor.setPower((int) (maxPower / 2) - (int) y);
			} else {
				curveL = true;
				rightMotor.setPower((int) (maxPower * 2 / 3));
				leftMotor.setPower((int) (-maxPower / 5));
			}
			/**
			 * right curve
			 */
		} else if ((e < 0) || curveR) {
			if (curveL) {
				y = PID_control(e, 0.15, 0.02, 0.02, 1);
				curveL = false;
				rightMotor.setPower((int) (maxPower / 2) - (int) y);
				leftMotor.setPower((int) (maxPower / 2) + (int) y);
			} else {
				curveR = true;
				rightMotor.setPower((int) (-maxPower / 5));
				leftMotor.setPower((int) (maxPower * 2 / 3));
			}
		}

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
	 *            angle velocity of the robot left turn is mathematical positive
	 *            right turn is mathematical negative
	 */
	private void drive(double v, double omega) {
		// Aufgabe 3.2
		double radiusM = 0; // radius of rotation; r=0--> rotate without
							// translation; r->inf -->straight driving

		double speed = 0;
		double vLeft = 0;
		double vRight = 0;
		int pwmLeft = 0;
		int pwmRight = 0;

		double disTurn = Math.PI * wheelD; // for regulated driving?
		double degTurn = disTurn / 360;
		double degSpeed = (v / degTurn) * 100;
		/**
		 * maxSpeed in deg/sec = 100deg/sec * battery.getVoltage maxSpeed/360 =
		 * maxTurn/sec maxTurn/sec * circumfance = maxSpeed in cm/sec
		 */
		double maxSpeed = ((Battery.getVoltage() * 100) / 360) * Math.PI
				* wheelD;

		/**
		 * If omega is zero, dont rotate; translate with both wheels turning
		 * with v
		 */
		if (omega == 0) {
			vLeft = v; // needs to be improved with regulated straight
						// driving
			vRight = v;
		}
		/**
		 * If v is zero, dont translate; rotate left if omega is positive and
		 * right if omega is negative
		 * 
		 */
		else if (v == 0) { // needs to be improved with regulated straight
							// driving
			speed = omega * width / 2; // turn around the middle of the robot
			vLeft = -speed;
			vRight = speed;
		}
		/**
		 * this should work for both orientations because radiusM and omega are
		 * negative on a right turn and therefore the sign of width/2 changes
		 * according to the orientation of the rotation
		 */
		else {
			radiusM = v / omega;
			speed = (radiusM - width / 2) * omega;
			vLeft = speed;
			speed = (radiusM + width / 2) * omega;
			vRight = speed;

		}
		pwmLeft = getPWM(vLeft);
		pwmRight = getPWM(vRight);

		// regWheelSpeed(vLeft, vRight);

		leftMotor.setPower(pwmLeft);
		rightMotor.setPower(pwmRight);

	}

	private int getPWM(double v) {
		int pwm = 0;
		int batteryLoadFull = 7700;
		// int batteryLoadHalf = 7500;
		/**
		 * measurements for full and half load showed only small deviations in
		 * the rise of the functions PWM(v) greater differences in PWM(omega)
		 * power loss when battery is very low-->voltage collapses
		 */
		if (Battery.getVoltageMilliVolt() > batteryLoadFull) {
			pwm = (int) (3.9 * v - 1);
		} else {
			pwm = (int) (3.8 * v - 1.3);
		}
		return pwm;
	}

	/**
	 * rotation speed regulation
	 */
	private void regWheelSpeed(double vL, double vR) {
		/**
		 * get angledifference in deg and time in msec
		 */
		double leftAngleDiff = this.encoderLeft.getEncoderMeasurement()
				.getAngleSum();
		double leftTime = this.encoderLeft.getEncoderMeasurement().getDeltaT() * 1000;
		double rightAngleDiff = this.encoderRight.getEncoderMeasurement()
				.getAngleSum();
		double rightTime = this.encoderRight.getEncoderMeasurement()
				.getDeltaT() * 1000;

		/**
		 * idea: get the average speed of the individual wheels with deltaPhi
		 * and deltaT
		 * 
		 */
		double omegaLeft = leftAngleDiff / leftTime;
		double omegaRight = rightAngleDiff / rightTime;

		/**
		 * v = w * r; actual value
		 */
		double vLeft = omegaLeft * wheelD / 2;
		double vRight = omegaRight * wheelD / 2;

		/**
		 * Definition of control error; individual regulation of each wheel vL
		 * and vR are the desired values
		 */
		double errVLeft = vL - vLeft;
		double errVRight = vR - vRight;

		/**
		 * calculate control values with new PID_control(e, KP, KI, KD, Ta)
		 */
		esumL = esumL + errVLeft;
		esumR = esumR + errVRight;

		/**
		 * TODO : test and get fitting values for KP, KI, KD
		 */
		double yL = KP * errVLeft + KI * esumL + KD * (e - ealtL);
		double yR = KP * errVRight + KI * esumR + KD * (e - ealtR);

		ealtL = errVLeft;
		ealtR = errVRight;

		double vLNew = vL + yL;
		double vRNew = vR + yR;

		/**
		 * calculate PWM values and set speed
		 */
		int pwmLeft = 0;
		int pwmRight = 0;
		pwmLeft = getPWM(vLNew);
		pwmRight = getPWM(vRNew);
		leftMotor.setPower(pwmLeft);
		rightMotor.setPower(pwmRight);

	}

	private double PID_control(double e, double KP, double KI, double KD,
			double Ta) {

		double y = 0; // Stellwert
		esum = esum + e;
		y = KP * e + KI * esum * Ta + KD * (e - ealt) / Ta;
		ealt = e;
		return y;
	}

	private void resetIntegralPID() {
		esum = 0;
	}

	private void Control_Demo_1() {
		//evtl Sound ausgeben bei ŸbergŠngen
		
		/**
		 * 120 cm with 10 cm/s straight driving
		 */
		double x = navigation.getPose().getX()*100;
		double y = navigation.getPose().getY()*100;
		double dis = Math.sqrt(x*x + y*y);
				
		if (demo1) {
			drive(10, 0);
			if(dis>=100){
			demo2 = true;
			demo1 = false;
			startAngle = (int) (navigation.getPose().getHeading()/Math.PI*180);
			}
		}

		/**
		 * 90 deg turn with 15deg/sec
		 */
		else if (demo2) {
			drive(0, 15);
			if((int) (navigation.getPose().getHeading()/Math.PI*180) - startAngle >= 90)
			demo3 = true;
			demo2 = false;
			startDis = dis;
		}

		/**
		 * 30 cm with 5 cm/s straight driving
		 */
		else if (demo3) {
			drive(5, 0);
			if((dis - startDis)>=30)
			{
			demo4 = true;
			demo3 = false;
			startAngle = (int) (navigation.getPose().getHeading()/Math.PI*180);
			}
		}

		/**
		 * -90 deg turn with 15deg/sec
		 */
		else if (demo4) {
			drive(0, -30);
			if((int) (navigation.getPose().getHeading()/Math.PI*180) - startAngle <= -90)
			{
			demo5 = true;
			demo4 = false;
			}
		}

		/**
		 * linecontrol
		 */
		else if (demo5) {
			exec_LINECTRL_ALGO();
			demo1 = true;
			demo5 = false;
		}
	}
}
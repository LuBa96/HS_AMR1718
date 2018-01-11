package parkingRobot;

import lejos.robotics.navigation.Pose;

/**
 * interface for the main module 'Control', providing methods for executing the algorithms to
 * control the robot according to different modes which is set by guidance.
 * 
 * @author RST
 */
public interface IControl {
	// Inputs
	
	
	/**
	 * The predefined control modes
	 */
	public enum ControlMode {
		
		/**
		 * folgt gerader Linie
		 */
		DEMO_STRAIGHT,
		
		/**
		 * fährt Kurve
		 */
		DEMO_TURN,
		
		/**
		 * folgt schwarzer Linie
		 */
		LINE_CTRL,
		
		/**
		 * Linkskurve
		 */
		LEFT_CRV_CTRL,
		
		/**
		 * Rechtskrve
		 */
		RIGHT_CRV_CTRL,
		
		/**
		 * einparken
		 */
		PARK_CTRL,
		
		/**
		 * v/w-Control
		 */
		VW_CTRL,
		
		/**
		 * eine Zielpsoe anfahren
		 */
		SETPOSE,
		
		/**
		 * NXT in Ruhe versetzen
		 */
		INACTIVE
	}
		
	/**
	 * set the required speed
	 * 
	 * @param velocity the velocity of the robot to be set
	 */	
	public void setVelocity(double velocity);

	
	/**
	 *  set the required angular velocity
	 *  
	 * @param angularVelocity  the angular velocity to be set
	 */
	public void setAngularVelocity(double angularVelocity);

	/**
	 * set the destination to be driven to
	 * 
	 * @param heading the heading angle of the robot at the destination
	 * @param x the destination position in x axis
	 * @param y the destination position in y axis
	 */
	public void setDestination(double heading, double x, double y);
		
	
	/**
	 * the Robot's current position 
	 * 
	 * @param currentPosition the current position of the robot at each sampling  
	 */	
	public void setPose(Pose currentPosition); 	
	
	/**
	 * update start position for angledifference and distance calculation
	 */
	public void updateStartPose();
	
	
	/**
	 * set the current control mode
	 * 
	 * @param ctrl_mode parameter for control mode which is defined by Guidance 
	 */
	public void setCtrlMode(ControlMode ctrl_mode);
	
	/**
	 * set start time
	 * @param startTime start time
	 */
	public void setStartTime(int startTime);
	
	
	/**
	 * execute the selected algorithms for control which was set by guidance
	 */
	public void exec_CTRL_ALGO();
	
	/**
	 * 
	 * @return true if robot has detected a turn
	 */
	public boolean getCurveMode();

	/**
	 * method for straight-turn transition
	 * @return true if robot has detected a left turn
	 */
	public boolean getLeftTurn();

	/**
	 * method for straight-turn transition
	 * @return true if robot has detected a right turn
	 */
	public boolean getRightTurn();


	/**
	 * reset accumulated error for PID calculations
	 */
	public void resetIntegralPID();
	
	/**
	 * reset accumulated error for RWD calculations
	 * RWD=regulated Wheel Control
	 */
	public void resetIntegralRWD();
	
	/**
	 * @return true if demo mode is finished
	 */
	public boolean getDemoStatus();
	
	/**
	 * 
	 * @param status
	 */
	public void setDemoStatus(boolean status);
	
	/**
	 * set Distance for straight driving
	 */
	public void setGoalDistance(double dis);
	
	/**
	 * set angle for turn
	 */
	public void setGoalAngle(double angle);
	
	/**
	 * sets and rotates the new Coordinate System before next DEMO step
	 * @param origin
	 */
	public void setCoSys(Pose origin);
	
	public double getYstrich();
	public double getXstrich();
	public double getesum();
	public Pose origin();
}


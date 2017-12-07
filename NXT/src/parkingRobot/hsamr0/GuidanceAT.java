package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.navigation.Pose;

import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;
import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;



/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements
	 * because they invoke different display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * Indicates that robot is following the line and detecting parking slots
		 */
		SCOUT,
		/**
		 * Indicates that robot is performing an parking maneuver
		 */
		DRIVING,
		/**
		 * Indicates the robot is performing a demo as part of the assignment for
		 * control
		 */
		DEMO,
		/**
		 * Indicates that the robot is currently standing still

		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}

	/**
	 * underlying states of the main state FOLLOW_LINE
	 */
	public enum CurrentLineStatus {
		/**
		 * Indicates the robot is following the line in a straight manner and there is
		 * no turn
		 */
		FOLLOW_LINE_STRAIGHT,
		/**
		 * Indicates the robot is taking a turn right
		 */
		FOLLOW_LINE_RIGHT,
		/**
		 * Indicates the robot is taking a turn left
		 */
		FOLLOW_LINE_LEFT,
		/**
		 * Indicates the robot is not on the line and is trying to find it
		 */
		FOLLOW_LINE_OFF,
		/**
		 * Indicates that the robot was previously not in the FOLLOW_LINE state
		 */
		FOLLOW_LINE_INACTIVE
	}

	/**
	 * underlying states of the main state PARK_THIS
	 */
	public enum CurrentParkStatus {
		/**
		 * Indicates the robot is driving along the line until destination is reached.
		 * This uses the four states of FOLLOW_LINE, but additionally checks whether a
		 * certain point on the map is reached, and if so, continues to drive into the
		 * parking slot following the path given by the path generator
		 */
		PARK_LINE_FOLLOW,
		/**
		 * Indicates the robot is following a path to get to a parking slot
		 */
		PARK_PATH_FOLLOW,
		/**
		 * Indicates the robot is correcting its position while already in the parking
		 * slot. Maybe we will need to seperate this into smaller steps/states.
		 */
		PARK_CORRECTING,
		/**
		 * Indicates that the robot was previously not in the PARK_THIS state
		 */
		PARK_INACTIVE
	}
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.DRIVING;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentLineStatus currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
	/**
	 * current underlying state of PARK_THIS
	 */
	protected static CurrentParkStatus currParkStatus = CurrentParkStatus.PARK_INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the
	 * actual state
	 */
	protected static CurrentStatus lastStatus = CurrentStatus.INACTIVE;
	/**
	 * last underlying status of FOLLOW_LINE
	 */
	protected static CurrentLineStatus lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
	/**
	 * last underlying status of PARK_THIS
	 */
	protected static CurrentParkStatus lastParkStatus = CurrentParkStatus.PARK_INACTIVE;

	/**
	 * one line of the map of the robot course. The course consists of a closed
	 * chain of straight lines. Thus every next line starts where the last line ends
	 * and the last line ends where the first line starts. This documentation for
	 * line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);

	static Line line4 = new Line(150, 30, 30, 30);
	static Line line5 = new Line(30, 30, 30, 60);
	static Line line6 = new Line(30, 60, 0, 60);
	static Line line7 = new Line(0, 60, 0, 0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight
	 * lines. Thus every next line starts where the last line ends and the last line
	 * ends where the first line starts. All above defined lines are bundled in this
	 * array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6, line7 };

	/**
	 * Point on the line-map the robot has to go in order to park, polynomial starts
	 * from around here.
	 */
	static Point mapGoal;
	/**
	 * The point in the center of the ParkingSlot, also the point the robot ends up
	 * by parking
	 */
	static Point slotGoal;
	/**
	 * Vector from the back to the front of the ParkingSlot, important in order to
	 * find the slotGoal and the goalPose.
	 */
	static Point slotDir;
	/**
	 * Number of the current selected ParkingSlot
	 */
	static int selectedParkingSlotNo = 0;
	/**
	 * The currently selected ParkingSlot
	 */
	static ParkingSlot selectedParkingSlot;
	/**
	 * The pose the robot has to end up by parking.
	 */
	static Pose goalPose;
	/**
	 * This tells us whether the robot should be off the line-map. This is important
	 * for determining which SCOUT sub-state has to be entered. It only gets set to
	 * true when the robot purposefully leaves the track, which happens when we want
	 * to park.
	 */
	static boolean offTrack = false;
	/**
	 * This tells us how far away from the mapGoal the robot will stop and start
	 * following a path into a parking slot. Needs to be tested.
	 */
	static double mapGoalDist = 15;
	/**
	 * This array holds the coefficients of the path polynomial
	 */
	static double[] coEffs;
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus = CurrentStatus.EXIT;

		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);
		
		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception, monitor);

		IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);

		monitor.startLogging();

		while (true) {
			showData(navigation, perception,control);

			switch (currentStatus) {
			case SCOUT:
				// MONITOR (example)
				// monitor.writeGuidanceComment("Guidance_Driving");

				// Into action
				if (lastStatus != CurrentStatus.SCOUT) {
					// this does not need to be here because of the transition check in the
					// SubStateMachine, but this way we save the time of one cycle
					if (offTrack)
						currLineStatus = CurrentLineStatus.FOLLOW_LINE_OFF;
					else
						currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;

					// activate parking slot detection, setOffTrack() in navigation has to be done
					// at some point too, probably when we change it
					navigation.setDetectionState(true);
				}

			// While action
			{
				// execute underlying state machine
				followLineSubStateMachine(control);
			}

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}

				// Leave action
				if (currentStatus != CurrentStatus.SCOUT) {
					// stop the robot(may not be needed)
					control.setCtrlMode(ControlMode.INACTIVE);
					// deactivate the underlying state machine
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
					lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
					// deactivate parking slot detection
					navigation.setDetectionState(false);
				}
				break;
			// There is no transition into this state yet.
			case PARK_THIS:
				// TODO implement parking

				// into action
				if (currentStatus != lastStatus) {
					// calculate our goal on the line-map and in the ParkingSlot
					selectedParkingSlotNo = hmi.getSelectedParkingSlot();
					selectedParkingSlot = navigation.getParkingSlots()[selectedParkingSlotNo];
					slotDir = selectedParkingSlot.getFrontBoundaryPosition()
							.subtract(selectedParkingSlot.getBackBoundaryPosition());
					// calculate goalPose from angle of slotDir here, before manipulating the
					// slotDir
					slotDir.multiplyBy((float) 0.5);
					slotGoal = selectedParkingSlot.getBackBoundaryPosition().add(slotDir);
					mapGoal = getClosestPointToGoal(slotGoal);

					currParkStatus = CurrentParkStatus.PARK_LINE_FOLLOW;
				}

			// while action
			{
				// execute sub-state machine
				parkThisSubStateMachine(control, navigation);
			}

				// state transition
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE) {
					currentStatus = CurrentStatus.INACTIVE;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				}

				// leave action
				if (currentStatus != lastStatus) {
					// deactivate the underlying state machine
					currParkStatus = CurrentParkStatus.PARK_INACTIVE;
					lastParkStatus = CurrentParkStatus.PARK_INACTIVE;
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
					lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
					control.setCtrlMode(ControlMode.INACTIVE);
				}
				break;
			case DEMO:
				// Into Action
				if (lastStatus != currentStatus)
					control.setCtrlMode(ControlMode.DEMO1_CTRL);

				// While Action

				// State transition check
				lastStatus = currentStatus;
				if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.INACTIVE;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (control.getDemoStatus()) {
					currentStatus = CurrentStatus.SCOUT;
				}
				break;
			case INACTIVE:
				// Into action
				if (lastStatus != CurrentStatus.INACTIVE) {
					control.setCtrlMode(ControlMode.INACTIVE);
				}

			// While action
			{
				// nothing to do here
			}

				// State transition check
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {
					currentStatus = CurrentStatus.SCOUT;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.SCOUT;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (Button.RIGHT.isDown()) {
					currentStatus = CurrentStatus.DEMO;
					while (Button.RIGHT.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				}

				// Leave action
				if (currentStatus != CurrentStatus.INACTIVE) {
					// nothing to do here
				}
				break;
			case EXIT:
				hmi.disconnect();
				/**
				 * NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE) //
				 * monitor.sendOfflineLog();
				 */
				monitor.stopLogging();
				System.exit(0);
				break;
			default:
				break;
        	}
        		
        	Thread.sleep(100);        	
		}
	}
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the
	 * requirements

	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return GuidanceAT.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */



	protected static void showData(INavigation navigation, IPerception perception, IControl control) {
		LCD.clear();

		LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);

		//LCD.drawString("left: " + (perception.getLeftLineSensorValue()), 0, 3);
		//LCD.drawString("right: " + (perception.getRightLineSensorValue()), 0, 4);
		// perception.showSensorData();
		LCD.drawString("X': " + (control.getYstrich()),0,3);
		LCD.drawString("Y': " + (control.getYstrich()),0,4);

		// if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
		// LCD.drawString("HMI Mode SCOUT", 0, 3);
		// }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
		// LCD.drawString("HMI Mode PAUSE", 0, 3);
		// }else{
		// LCD.drawString("HMI Mode UNKNOWN", 0, 3);
		// }
	}

	/**
	 * underlying state machine of FOLLOW_LINE, should do the same as without it at
	 * the moment. Will make the robot follow the line, which can also be used
	 * during PARK_THIS
	 */
	private static void followLineSubStateMachine(IControl control) {
		switch (currLineStatus) {
		case FOLLOW_LINE_STRAIGHT:
			// into action
			if (lastLineStatus != currLineStatus) {
				control.resetIntegralPID();
				control.resetIntegralRWD();
				control.setCtrlMode(ControlMode.LINE_CTRL);
			}
			// while action

			// state transitions
			lastLineStatus = currLineStatus;

			if (control.getRightTurn()) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_RIGHT;
			} else if (control.getLeftTurn()) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_LEFT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case FOLLOW_LINE_RIGHT:
			// TODO implement turning right

			// into action
			if (lastLineStatus != currLineStatus) {
				control.resetIntegralRWD();
				control.updateStartPose();
				control.setCtrlMode(ControlMode.RIGHT_CRV_CTRL);
			}

			// while action
			// TODO ask control whether it has finished driving the turn
			// TODO change transition booleans accordingly

			// state transitions
			lastLineStatus = currLineStatus;

			if (!control.getRightTurn()) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case FOLLOW_LINE_LEFT:
			// TODO implement turning left

			// into action
			if (lastLineStatus != currLineStatus) {
				control.updateStartPose();
				control.resetIntegralRWD();
				control.setCtrlMode(ControlMode.LEFT_CRV_CTRL);
			}

			// while action
			// TODO ask control whether it has finished driving the turn
			// TODO change transition booleans accordingly

			// state transitions
			lastLineStatus = currLineStatus;

			if (!control.getLeftTurn()) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		// there is no transition into this state yet
		case FOLLOW_LINE_OFF:
			// into action
			// TODO find path to the line, tell control to drive along that path

			// while action
			// TODO ask control/perception whether the line is reached
			// TODO change transition accordingly

			// state transition
			lastLineStatus = currLineStatus;
			currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			// if(reachedLine)
			// currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		// this state is only executed once after the LINE_FOLLOW is set active
		case FOLLOW_LINE_INACTIVE:
			// into action

			// while action

			// state transition check
			lastLineStatus = currLineStatus;
			if (offTrack)
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_OFF;
			else
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			break;
		}
	}

	private static void parkThisSubStateMachine(IControl control, INavigation navigation) {
		switch (currParkStatus) {
		case PARK_LINE_FOLLOW:
			// into action
			if (lastParkStatus != currParkStatus) {
				// this does not need to be here because of the transition check in the
				// SubStateMachine, but this way we save the time of one cycle
				if (offTrack)
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_OFF;
				else
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// while action
			followLineSubStateMachine(control);

			// state transition
			lastParkStatus = currParkStatus;
			if (navigation.getPose().getLocation().distance(mapGoal) <= mapGoalDist) {
				currParkStatus = CurrentParkStatus.PARK_PATH_FOLLOW;
			}

			// leaving action
			if (currParkStatus != lastParkStatus) {
				// stop the robot(may not be needed)
				control.setCtrlMode(ControlMode.INACTIVE);
				// deactivate the underlying state machine
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
			}
			break;
		case PARK_PATH_FOLLOW:
			// into action
			if (lastParkStatus != currParkStatus) {
				coEffs = setPolynomial(navigation.getPose().getLocation(), slotGoal);
			}

			// while action

			// state transition

			// leaving action
			break;
		case PARK_CORRECTING:
			// into action

			// while action

			// state transition

			// leaving action
			break;
		case PARK_INACTIVE:
			// into action

			// while action

			// state transition

			// leaving action
			break;
		}
	}

	/**
	 * This function returns the closest point on the map in regards to the point
	 * given. This is used for determining the point the robot drives to before
	 * parking in a parking slot.
	 * 
	 * @param l_goal
	 *            Point to which the closest point on the map has to be found
	 * @return
	 */
	private static Point getClosestPointToGoal(Point l_goal) {
		double a = 0;
		// number of the line in map with the shortest distance to goal
		int lineNo = 0;
		// vector from starting point of desired line to goal
		Point vectorR0;
		// vector from starting point of desired line to point of shortest distance to
		// goal on line
		Point vectorR1;
		// vector from (0;0) to point of shortest distance to goal on line
		Point vectorR2;

		// find out the line with the shortest distance to goal
		a = map[0].ptSegDist(l_goal);
		for (int i = 1; i < map.length; i++) {
			if (map[i].ptSegDist(l_goal) < a)
				lineNo = i;
		}
		vectorR0 = l_goal.subtract(map[lineNo].getP1());
		vectorR1 = vectorR0.projectOn(map[lineNo]);
		vectorR2 = map[lineNo].getP1().add(vectorR1);
		return vectorR2;
	}

	private static double[] setPolynomial(Point startPoint, Point endPoint) {
		double a, b, c, d = 0;
		double x0 = (endPoint.getX() - startPoint.getX())/2;
		return null;
	}

	/**
	 * This function sets the Point 0,0 for the coordinate system the robot uses
	 * when following a path.
	 * 
	 * @param startPoint
	 *            The Point the path starts.
	 * @param endPoint
	 *            The Point the path ends.
	 */
	private static void setTransformedCoordinates(Point startPoint, Point endPoint) {

	}
}
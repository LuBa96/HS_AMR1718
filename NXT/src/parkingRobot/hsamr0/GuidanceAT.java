package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;

import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;

//TODO check how the monitor works
//TODO implement underlying states of PARK_THIS
//TODO maybe transition check if DEMO is finished from control
//TODO test the PATH_FOLLOW sub state, if getSelectedParkingSpot does not work yet, think of a good test case
//TODO implement following a path back to the line
//TODO find an algorithm to check whether the robot is on the line again

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students
 * of electrical engineering with specialization 'automation, measurement and
 * control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego
 * NXT system witch can perform parking maneuvers on an predefined course. To
 * fulfill the interdisciplinary aspect of this project the software structure
 * is divided in 5 parts: human machine interface, guidance, control, perception
 * and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be
 * controlled by one or more finite state machines (FSM). It may be advantageous
 * to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be
 * realized in one main module class. Every class (except guidance) has
 * additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module
 * class thread is only handled in a synchronized context to avoid inconsistent
 * or corrupt data!
 */
public class GuidanceAT {

	/**
	 * states for the main finite state machine. This main states are
	 * requirements because they invoke different display modes in the human
	 * machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * Indicates that robot is following the line and detecting parking
		 * slots
		 */
		SCOUT,
		/**
		 * Indicates that robot is performing an parking maneuver
		 */
		PARK_THIS,
		/**
		 * Indicates the robot is performing a demo as part of the assignment
		 * for control
		 */
		DEMO1, DEMO2, DEMO3, RESET,
		/**
		 * Indicates that the robot is currently standing still
		 */
		INACTIVE,
		/**
		 * Indicates that robot is shutting down
		 */
		EXIT
	}

	/**
	 * underlying states of the main state FOLLOW_LINE
	 */
	public enum CurrentLineStatus {
		/**
		 * Indicates the robot is following the line in a straight manner and
		 * there is no turn
		 */
		FOLLOW_LINE_STRAIGHT,
		/**
		 * The first part of a turn, where the robot first needs to approach it
		 * a bit more.
		 */
		FOLLOW_LINE_TURN_STRAIGHT,
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
		 * Indicates the robot is on the line and correcting his heading.
		 */
		FOLLOW_LINE_CORRECT,
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
		 * Indicates the robot is driving along the line until destination is
		 * reached. This uses the four states of FOLLOW_LINE, but additionally
		 * checks whether a certain point on the map is reached, and if so,
		 * continues to drive into the parking slot following the path given by
		 * the path generator
		 */
		PARK_LINE_FOLLOW,
		/**
		 * Indicates the robot is following a path to get to a parking slot
		 */
		PARK_PATH_FOLLOW,
		/**
		 * Indicates the robot is correcting its position while already in the
		 * parking slot. Maybe we will need to seperate this into smaller
		 * steps/states.
		 */
		PARK_CORRECTING,
		/**
		 * Indicates that the robot was previously not in the PARK_THIS state
		 */
		PARK_INACTIVE
	}

	public enum demo1Status {
		DEMO_FIRST_LINE, DEMO_FIRST_TURN, DEMO_SECOND_LINE, DEMO_SECOND_TURN, DEMO_LINE_CONTROL, DEMO_CORRECTING, DEMO_INACTIVE
	}

	protected static demo1Status currDemo1Status = demo1Status.DEMO_INACTIVE;
	protected static demo1Status lastDemo1Status = demo1Status.DEMO_INACTIVE;

	public enum demo2Status {
		DEMO_DEMO1, DEMO_FIRST_TURN, DEMO_INACTIVE
	}

	protected static demo2Status currDemo2Status = demo2Status.DEMO_INACTIVE;
	protected static demo2Status lastDemo2Status = demo2Status.DEMO_INACTIVE;

	public enum demo3Status {
		DEMO_FIRST_SLOT, DEMO_SECOND_SLOT, DEMO_INACTIVE
	}

	protected static demo3Status currDemo3Status = demo3Status.DEMO_INACTIVE;
	protected static demo3Status lastDemo3Status = demo3Status.DEMO_INACTIVE;
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus = CurrentStatus.INACTIVE;
	/**
	 * current underlying state of FOLLOW_LINE
	 */
	protected static CurrentLineStatus currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
	/**
	 * current underlying state of PARK_THIS
	 */
	protected static CurrentParkStatus currParkStatus = CurrentParkStatus.PARK_INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering
	 * the actual state
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
	 * chain of straight lines. Thus every next line starts where the last line
	 * ends and the last line ends where the first line starts. This
	 * documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(0, 0, 180, 0);
	static Line line1 = new Line(180, 0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30, 30, 30);
	static Line line5 = new Line(30, 30, 30, 60);
	static Line line6 = new Line(30, 60, 0, 60);
	static Line line7 = new Line(0, 60, 0, 0);
	/**
	 * map of the robot course. The course consists of a closed chain of
	 * straight lines. Thus every next line starts where the last line ends and
	 * the last line ends where the first line starts. All above defined lines
	 * are bundled in this array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6,
			line7 };

	/**
	 * The Pose the robot is in at the start of each of his guidance loops
	 */
	static Pose currPose;
	/**
	 * Point on the line-map the robot has to go in order to park, polynomial
	 * starts from around here.
	 */
	static Point mapGoal;
	/**
	 * The point in the center of the ParkingSlot, also the point the robot ends
	 * up by parking
	 */
	static Point slotGoal;
	/**
	 * Vector from the back to the front of the ParkingSlot, important in order
	 * to find the slotGoal and the goalPose.
	 */
	static Point slotDir;
	/**
	 * The pose the robot has to end up by parking.
	 */
	static Pose goalPose;
	static boolean goalReached = false;
	static boolean rightTurn = false;
	static boolean leftTurn = false;
	static boolean turnStraightFin = true;
	static boolean turnFin = true;
	static boolean demo1Fin = false;
	static boolean demo2Fin = false;
	static boolean demo3Fin = false;
	static boolean wrongDir = false;
	/**
	 * Number of the current selected ParkingSlot
	 */
	static int selectedParkingSlotNo = 0;
	/**
	 * The currently selected ParkingSlot
	 */
	static ParkingSlot selectedParkingSlot;
	/**
	 * This tells us whether the robot should be off the line-map. This is
	 * important for determining which SCOUT sub-state has to be entered. It
	 * only gets set to true when the robot purposefully leaves the track, which
	 * happens when we want to park.
	 */
	static boolean offTrack = false;
	static Pose offTrackPose;
	static Pose startPose;
	/**
	 * This tells us how far away from the mapGoal the robot will stop and start
	 * following a path into a parking slot. Needs to be tested.
	 */
	static final double mapGoalDist = 30;
	static final double slotGoalDist = 1;
	static final double slotDegTol = 5;
	static final double offTrackDist = 2;
	static final double offTrackDeg = 2;
	static final double turnDist = 4.9;
	static final double turnDeg = 65;

	/**
	 * Greatest velocity with which the robot travels along the path into the
	 * parking slot.
	 */
	static final double vParkMax = 15;
	static double vPark;
	// number of the line in map with the shortest distance to goal
	static int lineNo = 0;

	static final double vLineMax = 40;
	static final double vLine0 = 10;
	static double vLine;
	static double distToMidMax;
	static double distToMid;
	static Point vectorA;
	static int counter;
	/**
	 * Robot moves forward if 1, backwards if -1
	 */
	static double direction = 1;
	/**
	 * This array holds the coefficients of the path polynomial
	 */
	static double[] coEffs;

	static double deltaPhiDeg;
	static double phiDot;
	/**
	 * The time in between two cycles of the thread.
	 */
	static double timePeriod = 100;

	static double currSysTime;
	static double lastSysTime;

	static CoordinateSystem coSys = null;

	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args
	 *            standard string arguments for main method
	 * @throws Exception
	 *             exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus = CurrentStatus.EXIT;

		// Generate objects

		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor,
				monitor);
		perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl control = new ControlRST(perception, navigation, leftMotor,
				rightMotor, monitor);
		INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);

		coSys = new CoordinateSystem();
		mapGoal = new Point(0, 0);
		slotGoal = new Point(0, 0);
		slotDir = new Point(0, 0);
		vectorA = new Point(0, 0);
		startPose = new Pose(0, 0, 0);
		offTrackPose = new Pose(0, 0, 0);
		goalPose = new Pose(0, 0, 0);
		currPose = new Pose(0, 0, 0);
		selectedParkingSlot = new ParkingSlot(0, mapGoal, mapGoal,
				ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING, 0, mapGoal, mapGoal);
		navigation.reset();

		// öffnen per cmd-Befehl: nxjconsole
		// RConsole.openUSB(15000);
		RConsole.println("Konsole initialisiert");

		monitor.startLogging();

		while (true) {
			currSysTime = System.currentTimeMillis();
			timePeriod = currSysTime - lastSysTime;
			lastSysTime = currSysTime;
			RConsole.println("Zykluszeit Guidance: "
					+ Double.toString(timePeriod));

			showData(navigation, perception, control);

			currPose.setLocation(navigation.getPose().getLocation()
					.multiply(100));
			currPose.setHeading(navigation.getPose().getHeading());

			switch (currentStatus) {
			case SCOUT:
				// MONITOR (example)
				// monitor.writeGuidanceComment("Guidance_Driving");

				// Into action
				if (currentStatus != lastStatus) {
					
				}

				// While action
				{
					// execute underlying state machine
					followLineSubStateMachine(control, navigation);
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
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS) {
					currentStatus = CurrentStatus.PARK_THIS;
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
			case PARK_THIS:
				// TODO implement parking
				// into action
				if (currentStatus != lastStatus) {
					RConsole.println("PARK_THIS startet");
					goalReached = false;
					// calculate our goal on the line-map and in the ParkingSlot
					selectedParkingSlotNo = hmi.getSelectedParkingSlot();
					selectedParkingSlot = navigation.getParkingSlots()[selectedParkingSlotNo - 1];
					slotDir = selectedParkingSlot.getFrontBoundaryPositionM()
							.subtract(
									selectedParkingSlot
											.getBackBoundaryPositionM());

					// // TODO:
					// calculate goalPose from angle of slotDir here, before
					// manipulating the
					// slotDir
					// RConsole.println("PARK_THIS startet immernoch");
					// slotDir.setLocation(30, 0);
					goalPose.setHeading(slotDir.angle());
					slotDir.multiplyBy((float) 0.5);
					slotGoal = selectedParkingSlot.getBackBoundaryPositionM()
							.add(slotDir);
					// slotGoal.setLocation(100, -30);
					mapGoal = getClosestPointToGoal(slotGoal);
					goalPose.setLocation(slotGoal);
					// currParkStatus = CurrentParkStatus.PARK_LINE_FOLLOW;
				}

				// while action
				{
					// execute sub-state machine
					parkThisSubStateMachine(control, navigation);
				}

				// state transition
				lastStatus = currentStatus;
				if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE
						|| goalReached) {
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
			case DEMO1:
				// Into Action
				if (lastStatus != currentStatus) {
					demo1Fin = false;
					offTrack = true;
					navigation.setOffTrack(true);
					currDemo1Status = demo1Status.DEMO_FIRST_LINE;
				}

				// While Action
				demo1SubStateMachine(control, navigation);

				// State transition check
				lastStatus = currentStatus;
				if (Button.ENTER.isDown() || demo1Fin) {
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

				}
				break;
			case DEMO2:
				// into action
				if (currentStatus != lastStatus) {
					demo2Fin = false;
					offTrack = true;
					navigation.setOffTrack(true);
					currDemo2Status = demo2Status.DEMO_DEMO1;
				}
				// while action
				demo2SubStateMachine(control, navigation);

				// transition check
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
				} else if (demo2Fin) {
					currentStatus = CurrentStatus.SCOUT;
				}
				// leave action
				if (currentStatus != lastStatus) {

				}
				break;
			case DEMO3:
				// into action
				if (currentStatus != lastStatus) {
					demo3Fin = false;
					//currDemo3Status = demo3Status.DEMO_FIRST_SLOT;
				}
				// while action
				demo3SubStateMachine(control, navigation);
				// state transition
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
				} else if (demo3Fin) {
					currentStatus = CurrentStatus.SCOUT;
				}
				// leave action
				if (currentStatus != lastStatus) {

				}
				break;
			// TODO
			case RESET:
				//into action
				if(currentStatus != lastStatus){
				navigation.reset();
				coSys = new CoordinateSystem();
				mapGoal = new Point(0, 0);
				slotGoal = new Point(0, 0);
				slotDir = new Point(0, 0);
				vectorA = new Point(0, 0);
				startPose = new Pose(0, 0, 0);
				offTrackPose = new Pose(0, 0, 0);
				goalPose = new Pose(0, 0, 0);
				currPose = new Pose(0, 0, 0);
				selectedParkingSlot = new ParkingSlot(0, mapGoal, mapGoal,
						ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING, 0, mapGoal,
						mapGoal);

				goalReached = false;
				rightTurn = false;
				leftTurn = false;
				turnStraightFin = true;
				turnFin = true;
				demo1Fin = false;
				demo2Fin = false;
				demo3Fin = false;
				wrongDir = false;
				offTrack = false;
				
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				currParkStatus = CurrentParkStatus.PARK_INACTIVE;
				lastParkStatus = CurrentParkStatus.PARK_INACTIVE;
				currDemo1Status = demo1Status.DEMO_INACTIVE;
				lastDemo1Status = demo1Status.DEMO_INACTIVE;
				currDemo2Status = demo2Status.DEMO_INACTIVE;
				lastDemo2Status = demo2Status.DEMO_INACTIVE;
				currDemo3Status = demo3Status.DEMO_INACTIVE;
				lastDemo3Status = demo3Status.DEMO_INACTIVE;
				}
				
				//while action
				//state transition
				lastStatus = currentStatus;
				if(hmi.getMode() == parkingRobot.INxtHmi.Mode.DEMO2)
				currentStatus = CurrentStatus.INACTIVE;
				//leave action
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
				if (Button.ESCAPE.isDown()) {
					currentStatus = CurrentStatus.EXIT;
					while (Button.ESCAPE.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT) {
					currentStatus = CurrentStatus.EXIT;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT) {
					currentStatus = CurrentStatus.SCOUT;
				} else if (Button.ENTER.isDown()) {
					currentStatus = CurrentStatus.SCOUT;
					while (Button.ENTER.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS) {
					currentStatus = CurrentStatus.PARK_THIS;
				} else if (Button.LEFT.isDown()
						|| (hmi.getMode() == parkingRobot.INxtHmi.Mode.RESET)) {
					currentStatus = CurrentStatus.RESET;
					while (Button.LEFT.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (Button.RIGHT.isDown()
						|| (hmi.getMode() == parkingRobot.INxtHmi.Mode.DEMO1)) {
					currentStatus = CurrentStatus.DEMO1;
					while (Button.RIGHT.isDown()) {
						Thread.sleep(1);
					} // wait for button release
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DEMO2) {
					currentStatus = CurrentStatus.DEMO2;
				} else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DEMO3) {
					currentStatus = CurrentStatus.DEMO3;
				}

				// Leave action
				if (currentStatus != CurrentStatus.INACTIVE) {
					// nothing to do here
				}
				break;
			case EXIT:
				hmi.disconnect();
				/**
				 * NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
				 * // monitor.sendOfflineLog();
				 */
				monitor.stopLogging();
				System.exit(0);
				break;
			default:
				break;
			}

			Thread.sleep(50);
		}
	}

	/**
	 * returns the actual state of the main finite state machine as defined by
	 * the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus() {
		return GuidanceAT.currentStatus;
	}

	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation
	 *            reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation,
			IPerception perception, IControl control) {
		LCD.clear();

		// LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100),
		// 0, 0);
		// LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100),
		// 0, 1);
		// LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() /
		// Math.PI *
		// 180), 0, 2);
		// LCD.drawString(Boolean.toString(goalReached), 0, 3);
		// LCD.drawString(Double.toString(goalPose.getHeading()), 0, 4);
		// LCD.drawString(Double.toString(offTrackPose.getX()), 0, 5);
		// LCD.drawString(Double.toString(offTrackPose.getY()), 0, 6);
		LCD.drawString(
				"Back X8: "
						+ Double.toString(navigation.getParkplatz(8)
								.getBackBoundaryPosition().getX()), 0, 0);
		LCD.drawString(
				"Front X8: "
						+ Double.toString(navigation.getParkplatz(8)
								.getFrontBoundaryPosition().getX()), 0, 1);
		LCD.drawString(
				"Back X9: "
						+ Double.toString(navigation.getParkplatz(9)
								.getBackBoundaryPosition().getX()), 0, 2);
		LCD.drawString(
				"Front X9: "
						+ Double.toString(navigation.getParkplatz(9)
								.getFrontBoundaryPosition().getX()), 0, 3);
		LCD.drawString(
				"Back X10: "
						+ Double.toString(navigation.getParkplatz(10)
								.getBackBoundaryPosition().getX()), 0, 4);
		LCD.drawString(
				"Front X10: "
						+ Double.toString(navigation.getParkplatz(10)
								.getFrontBoundaryPosition().getX()), 0, 5);
		LCD.drawString(
				"Back X11: "
						+ Double.toString(navigation.getParkplatz(11)
								.getBackBoundaryPosition().getX()), 0, 6);
		LCD.drawString(
				"Front X11: "
						+ Double.toString(navigation.getParkplatz(11)
								.getFrontBoundaryPosition().getX()), 0, 7);

		// if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
		// LCD.drawString("HMI Mode SCOUT", 0, 3);
		// }else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
		// LCD.drawString("HMI Mode PAUSE", 0, 3);
		// }else{
		// LCD.drawString("HMI Mode UNKNOWN", 0, 3);
		// }
	}

	/**
	 * underlying state machine of FOLLOW_LINE, should do the same as without it
	 * at the moment. Will make the robot follow the line, which can also be
	 * used during PARK_THIS
	 */
	private static void followLineSubStateMachine(IControl control,
			INavigation navigation) {
		switch (currLineStatus) {
		case FOLLOW_LINE_STRAIGHT:
			// into action
			if (lastLineStatus != currLineStatus) {
				control.resetIntegralPID();
				control.resetIntegralRWD();
				control.setVelocity(0);
				control.setCtrlMode(ControlMode.LINE_CTRL);
				vLine = vLine0;
				if(currentStatus == CurrentStatus.SCOUT)
				navigation.setDetectionState(true);
			}
			// while action
			// TODO: control velocity
			// navigation.getLineNumber()
			// distToMid = 0;
			if (wrongDir)
				vectorA = map[navigation.getLineNumber()].getP1().subtract(
						map[navigation.getLineNumber()].getP2());
			else
				vectorA = map[navigation.getLineNumber()].getP2().subtract(
						map[navigation.getLineNumber()].getP1());
			vectorA.multiplyBy((float) 0.5);
			distToMidMax = vectorA.length();
			if (map[navigation.getLineNumber()].getP2().distance(
					currPose.getLocation()) > distToMidMax)
				vLine += 0.2;
			else
				vLine -= 0.2;
			if (vLine < vLine0)
				vLine = vLine0;
			control.setVelocity(vLine);

			// state transitions
			lastLineStatus = currLineStatus;

			if (control.getRightTurn()) {
				if (nearTurn(navigation)) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_TURN_STRAIGHT;
					rightTurn = true;
					leftTurn = false;
				}
			} else if (control.getLeftTurn()) {
				if (nearTurn(navigation)) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_TURN_STRAIGHT;
					rightTurn = false;
					leftTurn = true;
				}
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case FOLLOW_LINE_TURN_STRAIGHT:
			// into action
			if (currLineStatus != lastLineStatus) {
				if (turnStraightFin) {
//					startPose.setLocation(currPose.getLocation());
//					startPose.setHeading(currPose.getHeading());
//					turnStraightFin = false;
//					turnFin = false;
//					navigation.setDetectionState(false);
					control.setGoalDistance(turnDist);
					control.setCoSys(currPose);
					control.setDemoStatus(false);
					control.setCtrlMode(ControlMode.DEMO_STRAIGHT);
				}
				control.setAngularVelocity(0);
				control.setVelocity(4);
				//control.setCtrlMode(ControlMode.VW_CTRL);
			}
			// while action

			// state transitions
			lastLineStatus = currLineStatus;
//			if (currPose.getLocation().distance(startPose.getLocation()) >= turnDist) {
//				if (rightTurn) {
//					currLineStatus = CurrentLineStatus.FOLLOW_LINE_RIGHT;
//				} else if (leftTurn) {
//					currLineStatus = CurrentLineStatus.FOLLOW_LINE_LEFT;
//				}
//			}
			if(control.getDemoStatus()){
				if (rightTurn) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_RIGHT;
				} else if (leftTurn) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_LEFT;
				}
			}
			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
				turnStraightFin = true;
			}
			break;
		// TODO check how much of the turn the robot actually completed, so that
		// he can
		// pause while turning
		case FOLLOW_LINE_RIGHT:
			// TODO implement turning right

			// into action
			if (lastLineStatus != currLineStatus) {
				control.resetIntegralRWD();
				control.setAngularVelocity(-30);
				control.setVelocity(0);
				control.setCtrlMode(ControlMode.VW_CTRL);
			}

			// while action
			control.setAngularVelocity((-90 - (currPose.getHeading() - startPose
					.getHeading()) * (180 / Math.PI))
					/ (8 * timePeriod * 0.001));

			// state transitions
			lastLineStatus = currLineStatus;
			if ((currPose.getHeading() - startPose.getHeading())
					* (180 / Math.PI) <= -turnDeg) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}
			// if (!control.getRightTurn()) {
			// currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			// }

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.PositionskorrekturAnEcken();
				rightTurn = false;
				leftTurn = false;
				turnFin = true;
			}
			break;
		case FOLLOW_LINE_LEFT:
			// TODO implement turning left

			// into action
			if (lastLineStatus != currLineStatus) {
				control.updateStartPose();
				control.resetIntegralRWD();
				control.setAngularVelocity(30);
				control.setVelocity(0);
				control.setCtrlMode(ControlMode.VW_CTRL);
			}

			// while action
			control.setAngularVelocity((90 - (currPose.getHeading() - startPose
					.getHeading()) * (180 / Math.PI))
					/ (8 * timePeriod * 0.001));
			// state transitions
			lastLineStatus = currLineStatus;
			if ((currPose.getHeading() - startPose.getHeading())
					* (180 / Math.PI) >= turnDeg) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
				navigation.PositionskorrekturAnEcken();
				rightTurn = false;
				leftTurn = false;
				turnFin = true;
			}
			break;
		// there is no transition into this state yet
		case FOLLOW_LINE_OFF:
			// into action
			// TODO find path to the line, tell control to drive along that path
			if (lastLineStatus != currLineStatus) {
				Sound.systemSound(true, 1);
				// coSys.setPointOfOrigin(currPose);
				// coEffs =
				// setPolynomial(coSys.getTransformedPose(offTrackPose).getLocation());
				direction = -1;
				control.setAngularVelocity(0);
				control.setVelocity(0);
				control.setCtrlMode(ControlMode.VW_CTRL);
			}

			// while action
			// TODO ask control/perception whether the line is reached
			// TODO change transition accordingly
			computePhiDot(coSys.getTransformedPose(currPose), coEffs);
			// velocity control, the straighter the path the faster the robot
			vPark = vParkMax * (1 - Math.abs(deltaPhiDeg) / 15);
			if (vPark < 5)
				vPark = 5;
			phiDot = phiDot * (vPark / vParkMax);
			control.setAngularVelocity(phiDot);
			control.setVelocity(-vPark);

			// state transition
			lastLineStatus = currLineStatus;
			// if(reachedLine)
			// currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT
			if (currPose.getLocation().distance(offTrackPose.getLocation()) <= offTrackDist) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_CORRECT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case FOLLOW_LINE_CORRECT:
			// into action
			if (lastLineStatus != currLineStatus) {
				counter = 0;
				control.resetIntegralPID();
				control.resetIntegralRWD();
				control.setVelocity(vLine0);
				control.setCtrlMode(ControlMode.LINE_CTRL);
				if(currentStatus == CurrentStatus.SCOUT)
				navigation.setDetectionState(true);
			}

			// while action
			counter++;

			// state transition
			lastLineStatus = currLineStatus;
			if (counter >= 10) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// leave action
			if (currLineStatus != lastLineStatus) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
				direction = 1;
				offTrack = false;
				navigation.setOffTrack(false);
				navigation.setParkingDone();
			}
			break;
		// this state is only executed once after the LINE_FOLLOW is set active
		case FOLLOW_LINE_INACTIVE:
			// into action

			// while action

			// state transition check
			lastLineStatus = currLineStatus;
			if (offTrack) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_OFF;
			} else if (!turnStraightFin) {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_TURN_STRAIGHT;
			} else if (!turnFin) {
				if (rightTurn) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_RIGHT;
				} else if (leftTurn) {
					currLineStatus = CurrentLineStatus.FOLLOW_LINE_LEFT;
				}
			} else {
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}
			break;
		}
	}

	private static void parkThisSubStateMachine(IControl control,
			INavigation navigation) {
		switch (currParkStatus) {
		case PARK_LINE_FOLLOW:
			// into action
			if (lastParkStatus != currParkStatus) {
				// this does not need to be here because of the transition check
				// in the
				// SubStateMachine, but this way we save the time of one cycle
				// if (offTrack)
				// currLineStatus = CurrentLineStatus.FOLLOW_LINE_OFF;
				// else
				// currLineStatus = CurrentLineStatus.FOLLOW_LINE_STRAIGHT;
			}

			// while action
			followLineSubStateMachine(control, navigation);
			RConsole.println(Double.toString(currPose.getLocation().distance(
					mapGoal)));
			// state transition
			lastParkStatus = currParkStatus;
			if ((currPose.getLocation().distance(mapGoal) <= mapGoalDist)
					&& (navigation.getLineNumber() == lineNo)) {
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
				RConsole.println("PARK_PATH_FOLLOW gestartet!");
				coSys.setPointOfOrigin(goalPose);
				coEffs = setPolynomial(coSys.getTransformedPose(currPose)
						.getLocation());
				direction = 1;
				offTrackPose.setLocation(currPose.getLocation());
				// offTrackPose.setHeading(map[navigation.getLineNumber()].getP2()
				// .subtract(map[navigation.getLineNumber()].getP1()).angle());
				offTrackPose.setHeading(currPose.getHeading());
				offTrack = true;
				navigation.setOffTrack(true);
				control.setAngularVelocity(0);
				control.setVelocity(0);
				control.setCtrlMode(ControlMode.VW_CTRL);
				RConsole.println("PARK_PATH_FOLLOW into action abgeschlossen!");
			}

			// while action
			RConsole.println("Folge Pfad");
			computePhiDot(coSys.getTransformedPose(currPose), coEffs);
			// velocity control, the straighter the path the faster the robot
			vPark = vParkMax * (1 - Math.abs(deltaPhiDeg) / 15);
			if (vPark < 5)
				vPark = 5;
			phiDot = phiDot * (vPark / vParkMax);
			control.setAngularVelocity(phiDot);
			control.setVelocity(vPark);

			// state transition
			lastParkStatus = currParkStatus;
			if (currPose.getLocation().distance(slotGoal) <= slotGoalDist) {
				currParkStatus = CurrentParkStatus.PARK_CORRECTING;
			}
			// leaving action
			if (currParkStatus != lastParkStatus) {
				// stop the robot(may not be needed)
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case PARK_CORRECTING:
			// into action
			if (currParkStatus != lastParkStatus) {
				phiDot = -coSys.getTransformedHeading(currPose);
				if (phiDot < 10)
					phiDot = 10;
				else if (phiDot > 45)
					phiDot = 45;
				control.setAngularVelocity(phiDot);
				control.setGoalAngle(90);
				control.setVelocity(0);
				control.setCoSys(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_TURN);
			}

			// while action

			// state transition
			lastParkStatus = currParkStatus;
			if (control.getDemoStatus()) {
				currParkStatus = CurrentParkStatus.PARK_INACTIVE;
				goalReached = true;
			}
			// leaving action
			if (currParkStatus != lastParkStatus) {
				// stop the robot(may not be needed)
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case PARK_INACTIVE:
			// into action

			// while action

			// state transition
			// TODO maybe update to a distance from current goal, so that
			// pausing while
			// following a path is possible
			lastParkStatus = currParkStatus;
			if (currPose.getLocation().distance(mapGoal) >= mapGoalDist) {
				currParkStatus = CurrentParkStatus.PARK_LINE_FOLLOW;
			} else {
				currParkStatus = CurrentParkStatus.PARK_PATH_FOLLOW;
			}
			// leaving action
			break;
		}
	}

	private static void demo1SubStateMachine(IControl control,
			INavigation navigation) {
		switch (currDemo1Status) {
		case DEMO_FIRST_LINE:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				control.setVelocity(10);
				control.setAngularVelocity(0);
				control.setGoalDistance(120);
				control.setCoSys(currPose);
				coSys.setPointOfOrigin(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_STRAIGHT);
			}
			// while action
			// state transition
			lastDemo1Status = currDemo1Status;
			if (control.getDemoStatus()) {
				currDemo1Status = demo1Status.DEMO_FIRST_TURN;
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_FIRST_TURN:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				control.setAngularVelocity(15);
				control.setGoalAngle(90);
				control.setVelocity(0);
				control.setCoSys(currPose);
				coSys.setPointOfOrigin(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_TURN);
			}
			// while action
			// state transition
			lastDemo1Status = currDemo1Status;
			if (control.getDemoStatus()) {
				currDemo1Status = demo1Status.DEMO_SECOND_LINE;
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_SECOND_LINE:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				control.setVelocity(5);
				control.setAngularVelocity(0);
				control.setGoalDistance(30);
				control.setCoSys(currPose);
				coSys.setPointOfOrigin(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_STRAIGHT);
			}
			// while action
			// state transition
			lastDemo1Status = currDemo1Status;
			if (control.getDemoStatus()) {
				currDemo1Status = demo1Status.DEMO_SECOND_TURN;
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_SECOND_TURN:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				control.setAngularVelocity(-30);
				control.setGoalAngle(-90);
				control.setVelocity(0);
				control.setCoSys(currPose);
				coSys.setPointOfOrigin(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_TURN);
			}
			// while action
			// state transition
			lastDemo1Status = currDemo1Status;
			if (control.getDemoStatus()) {
				currDemo1Status = demo1Status.DEMO_CORRECTING;
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_CORRECTING:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				counter = 0;
				control.resetIntegralPID();
				control.resetIntegralRWD();
				control.setVelocity(vLine0);
				control.setCtrlMode(ControlMode.LINE_CTRL);
			}
			// while action
			counter++;
			// state transition
			lastDemo1Status = currDemo1Status;
			if (counter >= 20) {
				currDemo1Status = demo1Status.DEMO_LINE_CONTROL;
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_LINE_CONTROL:
			// into action
			if (lastDemo1Status != currDemo1Status) {
				wrongDir = true;
				navigation.setBackwards(true);
				offTrack = false;
				navigation.setOffTrack(false);
			}
			// while action
			followLineSubStateMachine(control, navigation);
			// state transition
			lastDemo1Status = currDemo1Status;
			if (control.getRightTurn()) {
				if (nearTurn(navigation)) {
					demo1Fin = true;
				}
			}
			// leave action
			if (currDemo1Status != lastDemo1Status) {

			}
			break;
		case DEMO_INACTIVE:
			// into action
			if (lastDemo1Status != currDemo1Status) {

			}
			// while action
			// state transition
			lastDemo1Status = currDemo1Status;
			// leave action
			if (currDemo1Status != lastDemo1Status) {

			}
			break;
		}
	}

	private static void demo2SubStateMachine(IControl control,
			INavigation navigation) {
		switch (currDemo2Status) {
		case DEMO_DEMO1:
			// Into Action
			if (lastDemo2Status != currDemo2Status) {
				demo1Fin = false;
				navigation.setOffTrack(true);
				currDemo1Status = demo1Status.DEMO_FIRST_LINE;
			}

			// While Action
			demo1SubStateMachine(control, navigation);

			// State transition check
			lastDemo2Status = currDemo2Status;
			if (demo1Fin) {
				currDemo2Status = demo2Status.DEMO_FIRST_TURN;
			}
			// leave action
			if (currDemo2Status != lastDemo2Status) {
				currDemo1Status = demo1Status.DEMO_INACTIVE;
				lastDemo1Status = demo1Status.DEMO_INACTIVE;
				navigation.setOffTrack(false);
			}

			break;
		case DEMO_FIRST_TURN:
			// into action
			if (lastDemo2Status != currDemo2Status) {
				control.setAngularVelocity(45);
				control.setGoalAngle(180);
				control.setVelocity(0);
				control.setCoSys(currPose);
				coSys.setPointOfOrigin(currPose);
				control.setDemoStatus(false);
				control.setCtrlMode(ControlMode.DEMO_TURN);
			}
			// while action
			// state transition
			lastDemo2Status = currDemo2Status;
			if (control.getDemoStatus()) {
				demo2Fin = true;
				navigation.reset();
			}
			// leave action
			if (currDemo2Status != lastDemo2Status) {
				control.setVelocity(0);
				control.setAngularVelocity(0);
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_INACTIVE:
			// state transition
			lastDemo2Status = currDemo2Status;
			currDemo2Status = demo2Status.DEMO_DEMO1;
			break;
		}
	}

	private static void demo3SubStateMachine(IControl control,
			INavigation navigation) {
		switch (currDemo3Status) {
		case DEMO_FIRST_SLOT:
			// into action
			if (currDemo3Status != lastDemo3Status) {
				RConsole.println("PARK_THIS startet");
				goalReached = false;
				// calculate our goal on the line-map and in the ParkingSlot
				goalPose.setHeading(0);
				//TODO
				slotGoal.setLocation(50, -30);
				mapGoal = getClosestPointToGoal(slotGoal);
				goalPose.setLocation(slotGoal);
			}

			// while action
			{
				// execute sub-state machine
				parkThisSubStateMachine(control, navigation);
			}

			// state transition
			currDemo3Status = lastDemo3Status;
			if (goalReached = true) {
				currDemo3Status = demo3Status.DEMO_SECOND_SLOT;
			}

			// leave action
			if (currDemo3Status != lastDemo3Status) {
				// deactivate the underlying state machine
				currParkStatus = CurrentParkStatus.PARK_INACTIVE;
				lastParkStatus = CurrentParkStatus.PARK_INACTIVE;
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_SECOND_SLOT:
			// into action
			if (currDemo3Status != lastDemo3Status) {
				RConsole.println("PARK_THIS startet");
				goalReached = false;
				// calculate our goal on the line-map and in the ParkingSlot
				goalPose.setHeading(slotDir.angle());
				slotGoal.setLocation(210, 35);
				mapGoal = getClosestPointToGoal(slotGoal);
				goalPose.setLocation(slotGoal);
			}

			// while action
			{
				// execute sub-state machine
				parkThisSubStateMachine(control, navigation);
			}

			// state transition
			currDemo3Status = lastDemo3Status;
			if (goalReached = true) {
				demo3Fin = true;
			}

			// leave action
			if (currDemo3Status != lastDemo3Status) {
				// deactivate the underlying state machine
				currParkStatus = CurrentParkStatus.PARK_INACTIVE;
				lastParkStatus = CurrentParkStatus.PARK_INACTIVE;
				currLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				lastLineStatus = CurrentLineStatus.FOLLOW_LINE_INACTIVE;
				control.setCtrlMode(ControlMode.INACTIVE);
			}
			break;
		case DEMO_INACTIVE:
			//into action
			//while acton
			//state transitions
			lastDemo3Status = currDemo3Status;
			currDemo3Status = demo3Status.DEMO_FIRST_SLOT;
			//leave action
			break;
		}
	}

	/**
	 * This function returns the closest point on the map in regards to the
	 * point given. This is used for determining the point the robot drives to
	 * before parking in a parking slot.
	 * 
	 * @param l_goal
	 *            Point to which the closest point on the map has to be found
	 * @return
	 */
	private static Point getClosestPointToGoal(Point l_goal) {
		double a = 0;
		// vector from starting point of desired line to goal
		Point vectorR0;
		// vector from starting point of desired line to point of shortest
		// distance to
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

	private static double[] setPolynomial(Point startPoint) {
		double[] A = { 0, 0 };
		double x0 = startPoint.getX();
		double y0 = startPoint.getY();
		A[0] = (-2 * y0) / (x0 * x0 * x0);
		A[1] = (3 * y0) / (x0 * x0);
		return A;
	}

	private static void computePhiDot(Pose currPose, double[] coEffs) {

		double vx = vParkMax * Math.cos(currPose.getHeading());
		double xNext = currPose.getX() + direction * vx * timePeriod * 0.001;
		double nextPhiRad = Math.atan(3 * coEffs[0] * xNext * xNext + 2
				* coEffs[1] * xNext);
		deltaPhiDeg = (nextPhiRad - currPose.getHeading()) * (180 / Math.PI);
		while (deltaPhiDeg > 180)
			deltaPhiDeg -= 360;
		while (deltaPhiDeg < -180)
			deltaPhiDeg += 360;
		phiDot = deltaPhiDeg / (timePeriod * 0.001);
	}

	private static boolean nearTurn(INavigation navigation) {
		if (wrongDir) {
			return (currPose.getLocation().distance(
					map[navigation.getLineNumber()].getP1())
					/ map[navigation.getLineNumber()].length() <= 0.4);
		} else {
			return (currPose.getLocation().distance(
					map[navigation.getLineNumber()].getP2())
					/ map[navigation.getLineNumber()].length() <= 0.4);
		}
	}
}
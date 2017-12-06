
package parkingRobot.hsamr0;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;

import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.hsamr10.NavigationThread;

//import test3.main;
import parkingRobot.IMonitor;
import java.lang.*;
import java.util.*;

/**
 * A executable basic example implementation of the corresponding interface
 * provided by the Institute of Automation with limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception}
 * implementation are used for periodically calculating the robots position and
 * corresponding heading angle (together called 'pose'). Neither any use of the
 * map information or other perception sensor information is used nor any
 * parking slot detection is performed, although the necessary members are
 * already prepared. Advanced navigation calculation and parking slot detection
 * should be developed and invented by the students.
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation {

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

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null; // encoder left ->
													// Gradaenderung des Rades
													// nach letztem Aufruf
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null; // encoder right ->
													// Gradaenderung des Rades
													// nach letztem Aufruf

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null; // Unterschied
																		// zu
																		// encoderSensor??
																		// zusätzlich
																		// noch
																		// Zeitdifferenz?
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry
	 * sensor to measure the ground displacement between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null; // Maussensor

	/**
	 * reference to data class for measurement of the mouse odometry sensor to
	 * measure the ground displacement between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null; // Unterschied
																		// zu
																		// mouseodo??

	/**
	 * distance from optical sensor pointing in driving direction to obstacle in
	 * mm
	 */
	double frontSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance = 0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to
	 * obstacle in mm
	 */
	double backSensorDistance = 0;
	/**
	 * distance from optical sensor pointing to the right side of robot to
	 * obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance = 0;

	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS = 0.031; // 0.031 only rough guess, to
													// be measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS = 0.031; // only rough guess, to be
													// measured exactly and
													// maybe refined by
													// experiments
	/**
	 * robot specific constant: distance between wheels
	 */

	static final double WHEEL_DISTANCE		= 	0.170; // only rough guess, to be measured exactly and maybe refined by experiments

	static final double CIRCUMFERENCE_OF_MOUSE_NXT_FULL_TURN = 0; //noch ausmessen!!
	
	static final double DISTANCE_FrontSideSensorToRobotCenter = 0; //in cm, noch ausmessen! (TODO) -> erledigt
	
	static final double DISTANCE_BackSideSensorToRobotCenter = 2; //in cm, noch ausmessen(TODO)!; positiv wenn Sensor vor Zentrum des Roboters ist 
	
	static final double DISTANCE_RobotCenterToBarrierK0K7 = 20; // Abstand zur Bande/Hindernissen fuer Strecke nach Kurvenpunkten 7 und 0 (in cm); Wert muesste stimmen, aber nochmal ueberpruefen (TODO)
	
	static final double DISTANCE_RobotCenterToBarrierK3 = 15; // Abstand zur Bande/Hindernissen  Kurvenpunkt 3 (in cm); Wert muesste stimmen, aber nochmal ueberpruefen (TODO)
	
	static final double MIN_SLOT_DISTANCE = 45;
	

	/**
	 * map array of line references, whose corresponding lines form a closed
	 * chain and represent the map of the robot course

	 */
	Line[] map = null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception = null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off
	 * (false)
	 */

	boolean parkingSlotDetectionIsOn = false;


	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	/** Variablen fuer Location Bestimmung mit Maussensor **/

	static final double CIRCUMFERENCE_OF_MOUSE_CIRCLE__NXT_FULL_TURN = 100; // noch
																			// ausmessen!!

	public double heading = 0;
	public double Pos_x = 0; // x- Komponente der aktuellen Position (bezogen
								// auf Position beim Einschalten)
	public double Pos_y = 0; // y- Komponente der aktuellen Position (bezogen
								// auf Position beim Einschalten)
	public double dPos_x = 0; // Aederung der Position in x Richtung
	public double dPos_y = 0; // Aenderung der Position in y Richtung
	public double dx = 0;
	public double dy = 0;
	public double dt = 0; // "" (in s)
	public double last_x = 0; // x Wert nach letzter Abfrage
	public double last_y = 0; // y Wert nach letzter Abfrage

	/**
	 * Falls sich der Roboter in einer Kurve befindet, entspricht der aktuelle
	 * Kurvenpunkt dieser Kurve. Befindet er sich hinter einer Kurve entspricht
	 * der aktuelel Kurvenpunkt weiterhin der vorher durchfahrenen Kurve. Für
	 * Numerierung der Kurvenpunkte, siehe Dropbox Ordner Navigation ->
	 * Kurvenpunktfestlegung
	 **/
	public int aktuellerKurvenpunkt = 7; // Startpunkt des Roboters entspricht
											// Kurvenpunkt 7

	/**
	 * fusionMatrix wird fuer fusionsFunktion der Pose benoetigt. Die Zeilen der
	 * Matrix stehen für die verschiedenen Sensoren wobei:Encorder (0),
	 * Maussensor (1), Kompass (2) Spaltenformat: Index fuer den Sensor
	 * (beginnend bei 0), vom Sensor ermittelte x-Koordinate, y-Koordidate,
	 * Blickrichtung ACHTUNG: Fuer die Ausgabe der Encodersensoren muessen x-
	 * und y- Koordinaten *100 und der Winkel /Math.PI*180 gerechnet werden
	 **/
	double[][] fusionMatrix = new double[3][4];
	boolean aktuellInKurve = false;
	double sensor; // wird fuer fusionsFunktion fuer Pose benoetigt
	private boolean off_track = false;

	private int anzahlRunde = 1; //wird in WinkelKorrekturZwischenEcken erhoeht
	
	/**Kurvenmatrix in der alle Kurvenpunkte sowie deren Toleranzbereich für die Zuordnung der aktuellen Position zu einem Kurvenpunkt gespeichert sind
	 * Spaltenformat: Kurvenindex(beginnend bei 0), xminBereich, xmaxBereich, yminBereich, ymaxBereich, xKurvenpunkt, yKurvenpunkt, Einfahrtwinkel in die nach dem aktuellen Kurvenpunkt folgende Kurve, (CloseToCurve auf schon mal auf true = 1, noch nicht auf true = 0)**/
	double[][] Kurvenmatrix = new double[8][9];
	
	/**parkingSlots enthaelt alle erfassten Parkluecken
	 * Der Konstruktor jeder Parkluecke benoetigt folgende Paramter:(int ID, Point backBoundaryPosition, Point frontBoundaryPosition, ParkingSlotStatus slotStatus, int slotMeasurementQuality)
	 * -> Definition von hinterem und vorderem Punkt: 
	 * Steht Roboter neben Parkluecke, befindet sich der hintere Punkt der Parkluecke in Fahrtrichtung gesehen hinter dem Roboter**/
	INavigation.ParkingSlot[] parkingSlots = new INavigation.ParkingSlot[15]; //15 Parkluecken mit Index 0 - 14 ansprechbar
	
	/**
	 * Initialisierung der Laufvariable ParkingSlot. Hier werden die Werte jedes neu gefundenen Slots gespeichert,
	 * bis diese als neues Objekt in die Datenbank aufgenommen werden.
	 * Zur Information: Dem Konstruktor beim erstellen eines ParkingSlots muessen folgende Parameter mitgegeben werden:
	 * 	(int ID, Point backBoundaryPosition, Point frontBoundaryPosition, ParkingSlotStatus slotStatus, int slotMeasurementQuality)**/
	INavigation.ParkingSlot currentParkingSlot = new ParkingSlot(0, null, null, null, 0); //bisher wird aus currentParkingSlot verwendet: backBoundaryPosition, frontBoundaryPosition
	
	/**Anzahl der bereits gefundenen Parkluecken**/
	int anzahlParkluecken = 0; 
	
//	/**Fuer Ueberpruefung notwendig, ob Parkluecke aktualisiert oder neu erstellt werden soll**/
//	boolean parklueckeExistiertBereits = false;
//	/**
//	
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 * Pose wird im Moment am Ende aller Berechnungen auf xResult, yResult und angleResult gesetzt

	 */
	Pose pose = new Pose(); // Posenobjekt aus der Navigation Klass aus Lejos
							// wird erzeugt

	/**
	 * fusionierte Position mit Ecken, aber ohne Geradenkorrektur
	 * (Eckenkorrektur sollte nicht beim parken deaktiviert sein) Aktuell:
	 * xResult, yResult und angleResult entsprechen Pose
	 **/
	double xResult = 0;
	double yResult = 0;
	double angleResult = 0;

	/**
	 * Fuer Karte: fusioniertePose mit Ecken- und Geradenkorrektur (sollte
	 * beides beim parken deaktiviert sein) xResultMap, yResultMap bereits in
	 * mm; angleReslutMap in °
	 **/

	double xResultMap = 0;
	double yResultMap = 0;
	double angleResultMap = 0;

	/** folgende Variablen werden für WinkelkorrekturZwischenEcken benoetigt **/
	private int anzahlDurchlaufeMittelwert = 1; // gibt Anzahl der aktuellen
												// Messwerte inklusive des neuen
												// Messwertes fuer Berechnung
												// des Mittelwertes an
	private double angleResultAktuellerMittelwert = 0;
	private boolean winkelSchonKorrigiert = false;

	/** folgende Variable wird fuer closeToCurve Methode fuer Luke benoetigt **/
	public boolean robotCloseToCurve = false;

	
	/**folgende Variablen werden fuer detectParkingSlot benoetigt**/
	private double backBoundarxFrontSensor = 0;
	private double backBoundaryFrontSensor = 0;
	private double backBoundarxBackSensor = 0;
	private double backBoundaryBackSensor = 0;
	private double frontBoundarxFrontSensor = 0;
	private double frontBoundaryFrontSensor = 0;
	private double frontBoundarxBackSensor = 0;
	private double frontBoundaryBackSensor = 0;
	private double frontBoundarxFrontSensorAktuell = 0;
	private double frontBoundaryFrontSensorAktuell = 0;
	private boolean backBoundaryDetektiert = false; //wird benoetigt um in frontBoundary Erkennungsteil reinzuspringen
	private boolean frontBoundaryDetektiert = false; //wird benoetigt um backBoundary nicht auf gleichem Streckenabschnitt erneut neu zu setzen
	private double parklueckenLaenge = 0;

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * perception object (to obtain the measurement information from) and starts
	 * the navigation thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor) {
		this.perception = perception;
		this.monitor = monitor;
		this.encoderLeft = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();
		off_track = false;

		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		navThread.start();
		/** Kurvendaten initialisieren **/
		Kurvenmatrix[0][0] = 0;
		Kurvenmatrix[0][1] = 90;
		Kurvenmatrix[0][2] = 270;
		Kurvenmatrix[0][3] = -20;
		Kurvenmatrix[0][4] = 15;
		Kurvenmatrix[0][5] = 180;
		Kurvenmatrix[0][6] = 0;
		Kurvenmatrix[0][7] = 90;
		Kurvenmatrix[0][8] = 1;
		Kurvenmatrix[1][1] = 165;
		Kurvenmatrix[1][2] = 195;
		Kurvenmatrix[1][3] = 15;
		Kurvenmatrix[1][4] = 105;
		Kurvenmatrix[1][5] = 180;
		Kurvenmatrix[1][6] = 60;
		Kurvenmatrix[1][7] = 180;
		Kurvenmatrix[1][8] = 1;
		Kurvenmatrix[2][0] = 2;
		Kurvenmatrix[2][1] = 120;
		Kurvenmatrix[2][2] = 165;
		Kurvenmatrix[2][3] = 45;
		Kurvenmatrix[2][4] = 80;
		Kurvenmatrix[2][5] = 150;
		Kurvenmatrix[2][6] = 60;
		Kurvenmatrix[2][7] = 270;
		Kurvenmatrix[2][8] = 1;
		Kurvenmatrix[3][0] = 3;
		Kurvenmatrix[3][1] = 90;
		Kurvenmatrix[3][2] = 165;
		Kurvenmatrix[3][3] = 15;
		Kurvenmatrix[3][4] = 45;
		Kurvenmatrix[3][5] = 150;
		Kurvenmatrix[3][6] = 30;
		Kurvenmatrix[3][7] = 180;
		Kurvenmatrix[3][8] = 1;
		Kurvenmatrix[4][0] = 4;
		Kurvenmatrix[4][1] = 15;
		Kurvenmatrix[4][2] = 90;
		Kurvenmatrix[4][3] = 15;
		Kurvenmatrix[4][4] = 45;
		Kurvenmatrix[4][5] = 30;
		Kurvenmatrix[4][6] = 30;
		Kurvenmatrix[4][7] = 90;
		Kurvenmatrix[4][8] = 1;
		Kurvenmatrix[5][0] = 5;
		Kurvenmatrix[5][1] = 15;
		Kurvenmatrix[5][2] = 50;
		Kurvenmatrix[5][3] = 45;
		Kurvenmatrix[5][4] = 80;
		Kurvenmatrix[5][5] = 30;
		Kurvenmatrix[5][6] = 60;
		Kurvenmatrix[5][7] = 180;
		Kurvenmatrix[5][8] = 1;
		Kurvenmatrix[6][0] = 6;
		Kurvenmatrix[6][1] = -20;
		Kurvenmatrix[6][2] = 15;
		Kurvenmatrix[6][3] = 15;
		Kurvenmatrix[6][4] = 80;
		Kurvenmatrix[6][5] = 0;
		Kurvenmatrix[6][6] = 60;
		Kurvenmatrix[6][7] = 270;
		Kurvenmatrix[6][8] = 1;
		Kurvenmatrix[7][0] = 7;
		Kurvenmatrix[7][1] = -20;
		Kurvenmatrix[7][2] = 90;
		Kurvenmatrix[7][3] = -20;
		Kurvenmatrix[7][4] = 15;
		Kurvenmatrix[7][5] = 0;
		Kurvenmatrix[7][6] = 0;
		Kurvenmatrix[7][7] = 0;
		Kurvenmatrix[7][8] = 1;
	}

	// Inputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map) {
		this.map = map;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn) {
		this.parkingSlotDetectionIsOn = isOn;
	}

	public int getAktuellenKurvenpunkt() {
		return aktuellerKurvenpunkt;
	}

	public boolean getAktuellInKurve() {
		return aktuellInKurve;
	}

	public boolean getWinkelSchonKorrigiert() {
		return winkelSchonKorrigiert;
	}

	public void setOffTrack(boolean isOn) {
		off_track = isOn;
	}

	// Class control

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation() {
		this.updateSensors();
		this.calculateLocation();
		// perception.getBackSensorDistance();
		// perception.getUOdmometryDiffernce();
		this.calculateLocationUsingMouseSensor(
				perception.getUOdmometryDiffernce(),
				perception.getVOdometryDifference());
		// this.calculateLocationUsingCompass();
		if (this.parkingSlotDetectionIsOn)

				this.detectParkingSlot();
				
		
		

			this.detectParkingSlot();


		// MONITOR (example)
		// monitor.writeNavigationComment("Navigation");
	}

	// Outputs

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose() {
		return this.pose;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return parkingSlots; //hier Feld zurueckgeben
	}

	// Private methods

	/**
	 * calls the perception methods for obtaining actual measurement data and
	 * writes the data into members
	 */
	private void updateSensors() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		this.angleMeasurementLeft = this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight = this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement = this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance = perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance = perception.getBackSensorDistance();
		this.backSideSensorDistance = perception.getBackSideSensorDistance();
	}

	public double getfusionMatrixContent(int line, int row) {
		return fusionMatrix[line][row];
	}

	public double getxResult() {
		return xResult;
	}

	public double getyResult() {
		return yResult;
	}

	public double getAngleResult() {
		return angleResult;
	}

	public double getxResultMap() {
		return xResultMap;
	}

	public double getyResultMap() {
		return yResultMap;
	}

	public double getAngleResultMap() {
		return angleResultMap;
	}

	public double getRMS() {
		return angleResultAktuellerMittelwert;
	}

	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation() {
		double leftAngleSpeed = this.angleMeasurementLeft.getAngleSum()
				/ ((double) this.angleMeasurementLeft.getDeltaT() / 1000); // degree/seconds
		double rightAngleSpeed = this.angleMeasurementRight.getAngleSum()
				/ ((double) this.angleMeasurementRight.getDeltaT() / 1000); // degree/seconds

		double vLeft = (leftAngleSpeed * Math.PI * LEFT_WHEEL_RADIUS) / 180; // velocity
																				// of
																				// left
																				// wheel
																				// in
																				// m/s
		double vRight = (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180; // velocity
																				// of
																				// right
																				// wheel
																				// in
																				// m/s
		double w = (vRight - vLeft) / WHEEL_DISTANCE; // angular velocity of
														// robot in rad/s

		Double R = new Double((WHEEL_DISTANCE / 2)
				* ((vLeft + vRight) / (vRight - vLeft)));

		double ICCx = 0;
		double ICCy = 0;

		double xResult = 0;
		double yResult = 0;
		double angleResult = 0;

		double deltaT = ((double) this.angleMeasurementLeft.getDeltaT()) / 1000;

		if (R.isNaN()) { // robot doesn't move
			xResult = this.pose.getX();
			yResult = this.pose.getY();
			angleResult = this.pose.getHeading();
		} else if (R.isInfinite()) { // robot moves straight forward/backward,
										// vLeft==vRight
			xResult = this.pose.getX() + vLeft
					* Math.cos(this.pose.getHeading()) * deltaT;
			yResult = this.pose.getY() + vLeft
					* Math.sin(this.pose.getHeading()) * deltaT;
			angleResult = this.pose.getHeading();
		} else {
			ICCx = this.pose.getX() - R.doubleValue()
					* Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue()
					* Math.cos(this.pose.getHeading());

			xResult = Math.cos(w * deltaT) * (this.pose.getX() - ICCx)
					- Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult = Math.sin(w * deltaT) * (this.pose.getX() - ICCx)
					+ Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult = this.pose.getHeading() + w * deltaT;
		}
		sensor = 0; // fuer Zuordnung im Array zu Encoder
		// LCD.drawString("Encoder: " + xResult + "/" + yResult + "/" +
		// angleResult, 0, 0);
		this.fusionOfPoseData(sensor, (double) xResult, (double) yResult,
				(double) angleResult);

	}

	private void calculateLocationUsingMouseSensor(double x_in, double y_in) { // muss
																				// oft
																				// genug
																				// aufgerufen
																				// werden

		double x = x_in; // im Koordinatensystem des Roboters in x Richtung
							// zurueckgelegter Wert; wird von Perception in mm
							// uebergeben
		double y = y_in; // ""

		// Winkel der Blickrichtung bestimmen

		// dx = x - last_x; //Umrechnung noetig, da Werte von Maussensor als
		// absolute Wert uebergeben weren
		// dy = y - dy;
		//
		// last_x = x_in;
		// last_y = y_in;

		dx = x;
		dy = y;

		double dheading = -dx / CIRCUMFERENCE_OF_MOUSE_CIRCLE__NXT_FULL_TURN
				* 2 * (double) Math.PI; // funktioniert nur beim Stehen,
										// Kreisbogenlaenge muss je nach dy
										// Anteil angepasst werden -> Nein doch
										// nicht! Ausschliesslich dx ist fuer
										// Drehung verantwortlich
		heading += dheading; // zum Testen heading einfach mit dx und dy
								// vorgeben, update: jetzt mit dx und last_dx
								// bestimmen
		// Ueberlaufe verhindern //fuer cos, sin Funktionen für 'Position
		// bestimmen' Ueberlauf nicht schlimm. Nur fuer Schoenheit
		while (heading > 2 * Math.PI) {
			heading -= 2 * Math.PI;
		}
		while (heading < 0) {
			heading += 2 * Math.PI;
		}
		// System.out.println("Heading:  " + heading/(2*Math.PI)*360 + "°");

		// Position bestimmen (bei Naeherung von Geradeausfahrt mit bestimmtem
		// Winkel)
		dPos_x = dy * ((double) Math.cos(heading)); // cos liefert nur double
													// werte -> convertierung in
													// double
		dPos_y = dy * ((double) Math.sin(heading));
		Pos_x += dPos_x;
		Pos_y += dPos_y;
		// System.out.println("Koordinaten     x      |       y:");
		// System.out.println("             " + Pos_x + "  |  " + Pos_y);
		// System.out.println();

		xResult = Pos_x;
		yResult = Pos_y;
		angleResult = heading;
		// LCD.drawString("Mouse: " + xResult + "/" + yResult + "/" +
		// angleResult, 0, 1);
		sensor = 1;
		this.fusionOfPoseData(sensor, (double) xResult, (double) yResult,
				(double) angleResult / (2 * Math.PI) * 360);
	}

	private void calculateLocationUsingCompass() {

		// hier kommt Programm rein
		xResult = 0;
		yResult = 0;
		angleResult = 0;
		// LCD.drawString("Compass: " + xResult + "/" + yResult + "/" +
		// angleResult, 0, 2);
		sensor = 2;
		this.fusionOfPoseData(sensor, (double) xResult, (double) yResult,
				(double) angleResult);

	}

	/**
	 * Sensordaten werden mit folgenden Zeilennummern in Matrix eingetragen:
	 * Encorder (0), Maussensor (1), Kompass (2)
	 **/
	private void fusionOfPoseData(double s, double x, double y, double angle) {

		
		if (angle > 0.8*2*Math.PI) {
			angle = angle - 2*Math.PI;
		}
		fusionMatrix[(int)s][0] = s;
		fusionMatrix[(int)s][1] = x;
		fusionMatrix[(int)s][2] = y;
		fusionMatrix[(int)s][3] = angle;
		
		
		/**Im Folgenden: Auswertung der Daten mit Unsicherheit + Gewichtung oder Entscheidung, dass nur bestimmte Daten verwendet werden
		 * xResult und yResult und angleResult stellen die fusionierte Position dar, entspricht Pose**/
	
		xResult = fusionMatrix[0][1]; //xResult entspricht fusionierter Location (hier nur Encorder) HIER KOMMEN FUSIONSMETHODEN HIN; rausgenommen:  + fusionMatrix[1][1] + fusionMatrix[2][1])/3 
		yResult = fusionMatrix[0][2]; //  + fusionMatrix[1][2] + fusionMatrix[2][2])/3
		angleResult = fusionMatrix[0][3]; //  + fusionMatrix[1][3] + fusionMatrix[2][3])/3 
	
		/** 
		 * Falls sich Roboter auf dem Track befindet (z.B. bei Parkplatzsuche) werden Informationen der Strecke genutzt um Genauigkeit der aktuellen Position zu verbessern.
		 * Mit PositionskorrekturAnEcken() werden x und y an Eckpunkten auf den Eckpunkt gesetzt.
		 * Mit WinkelkorrekturZwischenEcken() wird zuerst der Mittelwert der Abweichung der Blickrichtung (heading) gebildet und diese anschließend auf ungefaehr halber Strecke zwischen zwei Ecken zum aktuellen Blickwinkel aufaddiert/abgezogen.
		 * Mit PositionFuerTabletMitPositionskorrekturAufGeraden() werden x bzw. y fuer eine schoene Darstellung auf dem Tablet genullt. 
		 */
		
		if(!off_track) {
			this.ueberpruefenObAktuellInKurve(); //setzt aktuellInKurve auf true oder false
			this.PositionskorrekturAnEcken();
			this.WinkelkorrekturZwischenEcken();
			this.PositionFuerTabletMitPositionskorrekturAufGeraden(); // berechnet xResultMap und yResultMap
		}
		
		/** Fusionierte Position soll die Eckenkorrektur uebernehmen, diese wird dann spaeter auch auf Pose uebtertragen
		 * -> hier alle Sensoren eintragen welche die Eckenkorrektur uebernehmen sollen**/
		fusionMatrix[0][1] = xResult;
		fusionMatrix[0][2] = yResult;
		fusionMatrix[0][3] = angleResult;
		
	
		this.pose.setLocation((float)xResult, (float)yResult); //x und y werden bereits in setPoseMitPositionskorrektur gesetzt
		this.pose.setHeading((float)angleResult);		 		
	//	LCD.drawString("Fusinoniert: " + (this.getPose().getX()*100) + "/" + (this.getPose().getY()*100) + (this.getPose().getHeading()/Math.PI*180), 0, 4);

	}

	/**
	 * Um festzustellen ob sich der Roboter in einer Kurve befindet, gilt das
	 * Kriterium, dass der aktuelle Winkel um mehr als 40 Grad vom Winkel der
	 * Strecke vor der Kurvenabfahrt abweichen muss. Ob groeßer oder kleiner
	 * wurde nicht unterschieden, da nicht notwendig.
	 */
	public void ueberpruefenObAktuellInKurve() {
		if (this.getPose().getHeading() / Math.PI * 180 > Kurvenmatrix[aktuellerKurvenpunkt][7] + 25
				|| this.getPose().getHeading() / Math.PI * 180 < Kurvenmatrix[aktuellerKurvenpunkt][7] - 25) { // fusionMatrix[0][3];
																												// (this.getPose().getHeading()>
																												// 35)
																												// ||
																												// (this.getPose().getHeading()<
																												// -35)
			aktuellInKurve = true; // aktuell hier noch ein Problem bei ueber
									// einem Durchlauf, da bei ca. 340 Grad
									// erste Bedingung sofort erfuellt -> Winkel
									// nach Kurven entweder mit Methode
									// korrigieren oder absolut (Kompass)
		} else {
			aktuellInKurve = false;
		}
	}

	/**
	 * Da Streckenkurs bekannt, wird an jeder Kurve des Parkours eine Korrektur
	 * der Position vorgenommen. Dabei werden sowohl Encoder, Maussensor und die
	 * Pose-Positionen ueberschrieben Im Parkmodus greift diese Korrektur nicht
	 **/
	private void PositionskorrekturAnEcken() {
		if ((!this.off_track && aktuellInKurve)) { // && aktuellInKurve
													// booleansche Variable die
													// angibt ob man sich in
													// Kurve befindet
													// implementieren
			/** Aktuell gefahrene Kurve zuordnen */
			for (int i = 0; i < 8; i++) {
				if ((aktuellInKurve && xResult * 100 > Kurvenmatrix[i][1])
						&& (xResult * 100 < Kurvenmatrix[i][2])
						&& (yResult * 100 > Kurvenmatrix[i][3])
						&& (yResult * 100 < Kurvenmatrix[i][4])) {
					// System.out.println("Roboter befindet sich am Kurvenpunkt"
					// + i);
					aktuellerKurvenpunkt = i;
					xResult = Kurvenmatrix[i][5] / 100; // xResult hat Einheit
														// von 1/100*mm,
														// Kurvenmatrix hat
														// bereits Einheit mm
					yResult = Kurvenmatrix[i][6] / 100;
					aktuellInKurve = false;
					winkelSchonKorrigiert = false;

					angleResultAktuellerMittelwert = 0;
					anzahlDurchlaufeMittelwert = 1;
				
					/**sobald Kurve beendet, befindet man sich optimalerweise wieder in Geradeaus-Fahrt
					 * -> Blickrichtung koennte hier evt. korrigiert werden oder Kalibrierung fuer Kompass waere hier moeglich**/
					
				}
				else {
					/**Hier evt. Fehlerkorrektur (vorerst nicht wichtig)
					 * Info: Man gelangt in Schleife:
					 * - bei 7 von 8 Durchlaeufen, da aktuelle Position nur einem Bereich zugeordnet werden kann (1x If schleife, 7x else schleife)
					 * - wenn man sich angeblich in Kurve befindet, der aktuellen Position aber kein Kurvenpunkt zugeordnet werden kann
					 -> vorerst wird an aktueller Position keine Aenderung vorgenommen (else Schleife bleibt leer)**/

				}
			}
		}
	}

	/** Methode um Winkel durch Mittelwertbildung zwischen den Kurven zu optimieren. Fuer die xResultMap, yResultMap, angleResultMap Daten werden die Paramteter stumpf gesetzt.
	 * Hier soll durch Mittelwertbildung auf ca. halber Strecke der Winkel einmal um den vom Mittelwert abweichenden Wert korrigiert werden.
	 * Das Parameter aktuellerKurvenpunkt wird deshalb verwendet, weil dieser auch nach der Kurve noch auf dem letzten Kurvenpunkt bestehen bleibt.
	 * Dieser repraesentiert also ebenso die Strecke nach dem aktuellen Kurvenpunkt
	 * TODO: Parameter noch soweit anpassen wie Pendelbewegung nach Kurvenausfahrt zu erkennen
	 * mindestens 15 cm, besser 20 vor Kurve aufhoeren Mittelwert zu bilden und aktuelles Heading mit Mittelwert anzupassen**/
	// Bemerkung: Merkwuerdigerweise wurden die beiden Parameter 'angleResultAktuellerMittelwert' und 'anzahlDurchlaufeMittelwert' in der zweiten if-Schleife in jeder case Anwendung nicht korrekt gesetzt (obwohl winkelSchonKorrigiert korrekt geaendert wurde)
	// Das Setzen der Werte der beiden Paramter wurde nun in die Methode PositionskorrekturAnEcken() verlegt. Somit werden erst an jeder Ecke die Werte gesetzt (was voellig ausreichend ist)
	private void WinkelkorrekturZwischenEcken() {
		if (!off_track) {
			switch (aktuellerKurvenpunkt) {
			case 0: if(yResult*100 > 10 && yResult*100 < 30) { //waren 10, 30
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(yResult*100 > 15 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (90*Math.PI/180 - angleResultAktuellerMittelwert); //90 Grad muss in Einheit von angleResult geaendert werden
						winkelSchonKorrigiert = true; //wird in PositionskorrekturAnEcken Methode auf false gesetzt
					}
					break;
			case 1: if(xResult*100 < 175 && xResult*100 > 170) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(xResult*100 < 170 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (180*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
					}
					break;
			case 2: if(yResult*100 < 55 && yResult*100 > 50) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(yResult*100 < 50 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (270*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
					}
					break;
			case 3: if(xResult*100 < 140 && xResult*100 > 90) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(xResult*100 < 90 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (180*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
					}
					break;
			case 4: if(yResult*100 > 35 && yResult*100 < 40) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(yResult*100 > 40 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (90*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
					}
					break;
			case 5: if(xResult*100 < 25 && xResult*100 > 20) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(xResult*100 < 20 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (180*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
					}
					break;
			case 6: if(yResult*100 < 55 && yResult*100 > 25) {
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(yResult*100 < 25 && winkelSchonKorrigiert == false) {
						angleResult = angleResult + (270*Math.PI/180 - angleResultAktuellerMittelwert);
						winkelSchonKorrigiert = true; 
						anzahlRunde++;//nochmal ueberdenken ob hier richtiger Ort
					}
					break;
			case 7: if(xResult*100 > 30 && xResult*100 < 70) { 
						angleResultAktuellerMittelwert = angleResultAktuellerMittelwert*(anzahlDurchlaufeMittelwert-1)/anzahlDurchlaufeMittelwert + angleResult/anzahlDurchlaufeMittelwert;
						anzahlDurchlaufeMittelwert++;
					}
					if(xResult*100 > 70 && winkelSchonKorrigiert == false) {
						angleResult = angleResult - angleResultAktuellerMittelwert;
						winkelSchonKorrigiert = true; 
					}
					break;
			}
		}
	}
	
	/**xResultMap und yResultMap werden berechnet: Auf gerader Strecke wird entsprechendes x bzw. y genullt**/
	private void PositionFuerTabletMitPositionskorrekturAufGeraden() { //hat bei Wechsel auf case 0 nicht yPose hochgezaehlt
			
			switch (aktuellerKurvenpunkt){ 							//geaendert zu von this.getAktullenKurvenpunkt!! -> ueberpruefen obs noch funktioniert
			case 0: xResultMap = 180;
					yResultMap = yResult*100;
					angleResultMap = 90;
					break;
			case 2: xResultMap = 150;
					yResultMap = yResult*100;
					angleResultMap = 270;
					break;
			case 4:	xResultMap = 30;
					yResultMap = yResult*100;
					angleResultMap = 90;
					break;
			case 6: xResultMap = 0;
					yResultMap = yResult*100; 
					angleResultMap = 270;
					break;
			case 1: xResultMap = xResult*100;
					yResultMap = 60;
					angleResultMap = 180;
					break;
			case 3: xResultMap = xResult*100;
					yResultMap = 30;
					angleResultMap = 180;
					break;
			case 5: xResultMap = xResult;
					yResultMap = 60;
					angleResultMap = 180;
					break;
			case 7: xResultMap = xResult*100;
					yResultMap = 0;
					angleResultMap = 0;
					break;
			}
	}
	/**aktuell geht robotCloseToCurve bei 10 Abstand vor Kurve auf true 
	 * TODO: durch if-Bedingung und Off-Track Variable sicherstellen, dass Luke nicht im Parkmodus Abfrage taetigen kann **/
	public void updateRobotCloseToCurve() {								//ACHTUNG: Aktuelle Annahme -> wegen return kein break noetig
																// TODO: Luke und Gregor sagen, dass sie mit setCloseToCurve = false
		switch (aktuellerKurvenpunkt){ 							//geaendert zu von this.getAktullenKurvenpunkt!! -> ueberpruefen obs noch funktioniert
		case 0: if (yResult*100 > 40 && Kurvenmatrix[0][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[0][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 1: if (xResult*100 < 170 && Kurvenmatrix[1][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[1][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 2:	if (yResult*100 < 50 && Kurvenmatrix[2][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[2][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 3: if (xResult*100 < 45 && Kurvenmatrix[3][8] == (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[3][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 4: if (yResult*100 > 40 && Kurvenmatrix[4][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[4][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 5: if (xResult*100 < 20 && Kurvenmatrix[5][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[5][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 6: if (yResult*100 < 20 && Kurvenmatrix[6][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[6][8]++;
					break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}
		case 7:  if (xResult*100 > 170 && Kurvenmatrix[7][8]== (double)anzahlRunde) {
					robotCloseToCurve = true;
					Kurvenmatrix[7][8]++;
						break;
				}
				else {
					robotCloseToCurve = false;
					break;
				}


	/**
	 * Methode um Winkel durch Mittelwertbildung zwischen den Kurven zu
	 * optimieren. Fuer die xResultMap, yResultMap, angleResultMap Daten werden
	 * die Paramteter stumpf gesetzt. Hier soll durch Mittelwertbildung auf ca.
	 * halber Strecke der Winkel einmal um den vom Mittelwert abweichenden Wert
	 * korrigiert werden. Das Parameter aktuellerKurvenpunkt wird deshalb
	 * verwendet, weil dieser auch nach der Kurve noch auf dem letzten
	 * Kurvenpunkt bestehen bleibt. Dieser repraesentiert also ebenso die
	 * Strecke nach dem aktuellen Kurvenpunkt TODO: Parameter noch soweit
	 * anpassen wie Pendelbewegung nach Kurvenausfahrt zu erkennen
	 **/

	private void WinkelkorrekturZwischenEcken() {
		switch (aktuellerKurvenpunkt) {
		case 0:
			if (yResult * 100 > 10 && yResult * 100 < 30) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult > 30 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (90 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true; // wird in
												// PositionskorrekturAnEcken
												// Methode auf false gesetzt
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 1:
			if (xResult * 100 < 175 && yResult * 100 > 160) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult < 160 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (180 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 2:
			if (yResult * 100 < 55 && yResult * 100 > 35) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult < 35 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (270 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 3:
			if (xResult * 100 < 140 && xResult * 100 > 90) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult < 90 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (180 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 4:
			if (yResult * 100 > 35 && yResult * 100 < 55) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult > 55 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (90 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 5:
			if (xResult * 100 < 25 && yResult * 100 > 5) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult < 5 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (180 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 6:
			if (yResult * 100 < 55 && yResult * 100 > 25) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult < 25 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (270 - angleResultAktuellerMittelwert / Math.PI * 180);
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		case 7:
			if (xResult * 100 > 30 && xResult * 100 < 70) {
				angleResultAktuellerMittelwert = angleResultAktuellerMittelwert
						* (anzahlDurchlaufeMittelwert - 1)
						/ anzahlDurchlaufeMittelwert + angleResult
						/ anzahlDurchlaufeMittelwert;
			}
			if (yResult > 70 && winkelSchonKorrigiert == false) {
				angleResult = angleResult
						+ (90 - angleResultAktuellerMittelwert / Math.PI * 180)
						* Math.PI * 180;
				angleResultAktuellerMittelwert = 0;
				winkelSchonKorrigiert = true;
				anzahlDurchlaufeMittelwert = 0;
			}
			anzahlDurchlaufeMittelwert++;
			break;
		}
	}

	private void PositionFuerTabletMitPositionskorrekturAufGeraden() { // hat
																		// bei
																		// Wechsel
																		// auf
																		// case
																		// 0
																		// nicht
																		// yPose
																		// hochgezaehlt

		switch (aktuellerKurvenpunkt) { // geaendert zu von
										// this.getAktullenKurvenpunkt!! ->
										// ueberpruefen obs noch funktioniert
		case 0:
			xResultMap = 180;
			yResultMap = yResult * 100;
			angleResultMap = 90;
			break;
		case 2:
			xResultMap = 150;
			yResultMap = yResult * 100;
			angleResultMap = 270;
			break;
		case 4:
			xResultMap = 30;
			yResultMap = yResult * 100;
			angleResultMap = 90;
			break;
		case 6:
			xResultMap = 0;
			yResultMap = yResult * 100;
			angleResultMap = 270;
			break;
		case 1:
			xResultMap = xResult * 100;
			yResultMap = 60;
			angleResultMap = 180;
			break;
		case 3:
			xResultMap = xResult * 100;
			yResultMap = 30;
			angleResultMap = 180;
			break;
		case 5:
			xResultMap = xResult;
			yResultMap = 60;
			angleResultMap = 180;
			break;
		case 7:
			xResultMap = xResult * 100;
			yResultMap = 0;
			angleResultMap = 0;
			break;
		}
	}

	/**
	 * aktuell geht robotCloseToCurve bei 10 Abstand vor Kurve auf true TODO:
	 * durch if-Bedingung und Off-Track Variable sicherstellen, dass Luke nicht
	 * im Parkmodus Abfrage taetigen kann
	 **/
	public void updateRobotCloseToCurve() { // ACHTUNG: Aktuelle Annahme ->
											// wegen return kein break noetig
		// TODO: Luke und Gregor sagen, dass sie mit setCloseToCurve = false
		switch (aktuellerKurvenpunkt) { // geaendert zu von
										// this.getAktullenKurvenpunkt!! ->
										// ueberpruefen obs noch funktioniert
		case 0:
			if (yResult * 100 > 50) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 1:
			if (xResult * 100 < 140) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 2:
			if (yResult * 100 < 40) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 3:
			if (xResult * 100 < 40) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 4:
			if (yResult * 100 > 50) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 5:
			if (xResult * 100 < 10) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 6:
			if (yResult * 100 < 10) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}
		case 7:
			if (xResult * 100 > 170) {
				robotCloseToCurve = true;
				break;
			} else {
				robotCloseToCurve = false;
				break;
			}

		}
	}

	public boolean getRobotCloseToCurve() {
		return robotCloseToCurve;
	}

	public void setRobotCloseToCurveToFalse() {
		robotCloseToCurve = false;
	}

	/**
	 * detects parking slots and manage them by initializing new slots,
	 * re-characterizing old slots or merge old and detected slots.
	 */

	
	/** Methode zur Erkennung und Eintragung von Parkluecken in eine Datenbank
	 * Hinweis: Methode wird nur aufgerufen falls parkingSlotDetection auf true gesetzt
	 * Die Methode ermittelt nicht die vorderen Positionsecken der Parkluecke sondern die Positionen auf halber Tiefe in den Parkluecken!**/
	private void detectParkingSlot(){
	
//		double aktuellFrontSideSensorDistance = perception.getFrontSideSensorDistance();
		if(perception.getFrontSideSensorDistance() > 250 && !backBoundaryDetektiert && !frontBoundaryDetektiert) {	//   Parkluecken nach Kurvenpunkten 7 und 0 sind mit 20cm am weitesten entfernt. 5 cm Toleranz eingeplant (evt. nicht aussreichend)
			backBoundaryDetektiert = true;
			switch(aktuellerKurvenpunkt) {
				case 0: backBoundarxFrontSensor = this.pose.getX()*100 -DISTANCE_RobotCenterToBarrierK0K7; //y Position gut, fuer x Position evt. ein spaeteren Wert nehmen, da zu frueh ermittelt und evt noch an Kante
						backBoundaryFrontSensor = this.pose.getY()*100 + DISTANCE_FrontSideSensorToRobotCenter;
						Point backBoundarPoint = new Point((float)backBoundarxFrontSensor, (float)backBoundaryFrontSensor); 
						currentParkingSlot.setBackBoundaryPosition(backBoundarPoint);	//diesen Umweg damit Punkt nicht global gespeichert sondern in currentParkingSlot als Attribut gespeichert wird
						break;
				case 1: case 2 : case 4: case 5: case 6: //break, da es keine Parkluecken auf diesen Strecken geben sollte -> hier wuerde 
						LCD.drawString("Auf unerwartetem Streckenabschnitt Parkluecke entdeckt",0,5);
						break;
				case 3: backBoundarxFrontSensor = this.pose.getX()*100 - DISTANCE_FrontSideSensorToRobotCenter;
						backBoundaryFrontSensor = this.pose.getY()*100 + DISTANCE_RobotCenterToBarrierK3;
						break;
				case 7: backBoundarxFrontSensor = this.pose.getX()*100 + DISTANCE_FrontSideSensorToRobotCenter;
						backBoundaryFrontSensor = this.pose.getY()*100 - DISTANCE_RobotCenterToBarrierK0K7;
						break;
			}
		}
												
												
		/**Diese Abfrage soll die zwei wichtigen Punkte der Parkluecken bestimmen und unter frontBoundarxFrontSensor und frontBounaryFrontSensor abspeichern.
		 * Anschließend soll mit diesen Punkten gebprueft werden ob Parkluecke bereits bekannt -> aktualisieren
		 * 																	   zu klein -> nichts machen, evt Ausgaben: Parkluecke zu klein
		 * 																	   ausreichend gross-> in Datenbank aufnehmen**/						
		
//		aktuellFrontSideSensorDistance = perception.getFrontSideSensorDistance(); // Sensorwert aktualisieren damit noch im gleichen detectParkingSlot Aufruf in Abfrage reingesprungen werden kann. Bei geringem threadSleep eig. nicht unbedingt noetig			
		if (perception.getFrontSideSensorDistance() > 250 && backBoundaryDetektiert) { //Szene: Hinterer Parklueckenpunkt wurde vermerkert. Jetzt schauen ob Parkluecke gross genug			
			switch(aktuellerKurvenpunkt) {
				case 0: frontBoundarxFrontSensorAktuell = this.pose.getX()*100 -DISTANCE_RobotCenterToBarrierK0K7; // hierbei handelt es sich nur um vorlaeufige Positionen der Parkluecken
						frontBoundaryFrontSensorAktuell = this.pose.getY()*100 + DISTANCE_FrontSideSensorToRobotCenter;
						break;
				case 1: case 2 : case 4: case 5: case 6: //break, da es keine Parkluecken auf diesen Strecken gibt
						break;
				case 3: frontBoundarxFrontSensorAktuell = this.pose.getX()*100 - DISTANCE_FrontSideSensorToRobotCenter;
						frontBoundaryFrontSensorAktuell = this.pose.getY()*100 + DISTANCE_RobotCenterToBarrierK3;
						break;
				case 7: frontBoundarxFrontSensorAktuell = this.pose.getX()*100 + DISTANCE_FrontSideSensorToRobotCenter;
						frontBoundaryFrontSensorAktuell = this.pose.getY()*100 - DISTANCE_RobotCenterToBarrierK0K7;
						break;
			}
		}
		if (perception.getFrontSideSensorDistance() < 250 && backBoundaryDetektiert) { //hier kommen wir rein wenn der vom Sensor erfasste Wert kleiner/gleich 25 ist, Parkluecke also vorbei ist (oder backBoundaryDetektiert auf false steht - am Ende das nochmal ueberpruefen ob ok)
			backBoundaryDetektiert = false;
			frontBoundaryDetektiert = true;
			frontBoundarxFrontSensor = frontBoundarxFrontSensorAktuell; //x Koordinate vom vorderen Parklueckenpunkt wird gespeichert 
			frontBoundaryFrontSensor = frontBoundaryFrontSensorAktuell; //y Koordinate vom hinteren Parklueckenpunkt wird gespeichert
			Point frontBoundarPoint = new Point((float)frontBoundarxFrontSensor, (float)frontBoundaryFrontSensor);
			currentParkingSlot.setFrontBoundaryPosition(frontBoundarPoint);
			currentParkingSlot.setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING); //vorerst einfach so festgelegt, spaeter evt nicht fix machen
			currentParkingSlot.setMeasurementQuality(1);						  //vorerst einfach so festgelegt, spaeter evt nicht fix machen
			/** Je nach Kurvenpunkt wird nun ueberprueft ob Parkluecke ausreichend gross ist und diese anschliessend aktualisiert bzw. hinzugefuegt**/
			switch(aktuellerKurvenpunkt) {
				case 0: if (frontBoundaryFrontSensor > (backBoundaryFrontSensor + MIN_SLOT_DISTANCE)) { //Parkluecke soll vorerst mindestens 45cm groß sein
							if(parklueckeExistiertBereits()) {
								ParklueckeAktualisieren(); //Im Falle, dass Parkluecke bereits existiert sollen die neu erfassten Boundarypositionen mit den alten fusioniert werden (Mittelwert)
								frontBoundaryDetektiert = false;
							}
							else {
								parkingSlots[anzahlParkluecken] = new ParkingSlot(anzahlParkluecken, currentParkingSlot.getBackBoundaryPosition(), currentParkingSlot.getFrontBoundaryPosition(), currentParkingSlot.getStatus(), currentParkingSlot.getMeasurementQuality()); //an Position +1 einfuegen, da currentParkingSlot bereits an Position 0 in Array -> falsch, currentParkingSlot sollte eigentlich nur ID von 0 haben, dem Feld aber nicht hinzugefuegt worden sein 
								anzahlParkluecken++;
								frontBoundaryDetektiert = false;
							}
						}
						break;
				case 1: case 2 : case 4: case 5: case 6:
						break;
				case 3: if (frontBoundarxFrontSensor < (backBoundarxFrontSensor - MIN_SLOT_DISTANCE)) { //Parkluecke soll vorerst mindestens 45cm groß sein
							if(parklueckeExistiertBereits()) {
									ParklueckeAktualisieren(); //Im Falle, dass Parkluecke bereits existiert sollen die neu erfassten Boundarypositionen mit den alten fusioniert werden (Mittelwert)
									frontBoundaryDetektiert = false;
							}
							else {
								parkingSlots[anzahlParkluecken] = new ParkingSlot(anzahlParkluecken, currentParkingSlot.getBackBoundaryPosition(), currentParkingSlot.getFrontBoundaryPosition(), currentParkingSlot.getStatus(), currentParkingSlot.getMeasurementQuality());
								anzahlParkluecken++;
								frontBoundaryDetektiert = false;
							}
						}
						break;
				case 7: parklueckenLaenge = frontBoundarxFrontSensor - backBoundarxFrontSensor;
					if (parklueckenLaenge > MIN_SLOT_DISTANCE) { //Parkluecke soll vorerst mindestens 45cm groß sein
							if(parklueckeExistiertBereits()) {
								ParklueckeAktualisieren(); //Im Falle, dass Parkluecke bereits existiert sollen die neu erfassten Boundarypositionen mit den alten fusioniert werden (Mittelwert)
								frontBoundaryDetektiert = false;
							}
							else {
							parkingSlots[anzahlParkluecken] = new ParkingSlot(anzahlParkluecken, currentParkingSlot.getBackBoundaryPosition(), currentParkingSlot.getFrontBoundaryPosition(), currentParkingSlot.getStatus(), currentParkingSlot.getMeasurementQuality());
							anzahlParkluecken++;
							frontBoundaryDetektiert = false;
							}
						}
						break;
				
			}	
		}
	}
	/** Hier wird ueberprueft ob Daten der aktuell hinzuzufuegenden Parkluecke ungefaehr mit einer bereits vorhandenen Parkluecke uebereinstimmen**/
	private boolean parklueckeExistiertBereits() {
		return false; //vorerst fuers Testen auf false, Aktualisierung der Parkluecke somit deaktiviert und auch nicht moeglich
	}
	
	private void ParklueckeAktualisieren() {
		//erst implementieren wenn Rest laeuft
	}
	
	/**Methoden nur fuer Ausgabe**/
	public boolean backBoundaryDetektiert() {
		return backBoundaryDetektiert;
	}
	public boolean frontBoundaryDetektiert() {
		return frontBoundaryDetektiert;
	}
	public double backBoundarxFrontSensor() {
		return backBoundarxFrontSensor;
	}
	public double backBoundaryFrontSensor() {
		return backBoundaryFrontSensor;
	}
	public double frontBoundarxFrontSensor() {
		return frontBoundarxFrontSensor;
	}
	public double frontBoundaryFrontSensor() {
		return frontBoundaryFrontSensor;
	}
	public double frontBoundarxFrontSensorAktuell() {
		return frontBoundarxFrontSensorAktuell;
	}
	public double frontBoundaryFrontSensorAktuell() {
		return frontBoundaryFrontSensorAktuell;
	}
	public double parklueckenLaenge() {
		return parklueckenLaenge;
	}
}
/**TODO:
	1. Bei setLocation xResult*100 und Winkel bereits richtig eintragen, damit Pose bereits in richtiger Einheit gespeichert wird
	
	2.Luke eine Variable die true gibt wenn 10 cm vor Kurve, sobald in Kurve aber sofort wieder auf true -> done
	4. Gregor Mittelpunkt liefern
	3. Parkluecke erkennen 
	5. T Anfangspunkte geben, nicht mitte!
	6. Ueberpruefen ob Parkplaetze halbwegs korrekt eingetragen werden
	7. Mit Luke und Gregor ueber Loesung reden
	8. setParkingDetection auf false machen und ueberpruefen ob T es aendern kann
**/

/** Ermittlung der Parklueckenpunkte fuer Gregor mit Verwendung der Mitte zwischen hinterstem (tiefstem Punkt) der Parkluecke und vorderem Bandenpunkt:


switch(aktuellerKurvenpunkt) {
case 0: backBoundarxFrontSensor = this.pose.getX()*100 + DISTANCE_RobotCenterToBarrier + (aktuellFrontSideSensorDistance -DISTANCE_RobotCenterToBarrier)/2; //y Position gut, fuer x Position evt. ein spaeteren Wert nehmen, da zu frueh ermittelt und evt noch an Kante
		backBoundaryFrontSensor = this.pose.getY()*100 + DISTANCE_FrontSideSensorToRobotCenter;
case 1: case 2 : case 4: case 5: case 6: //break, da es keine Parkluecken auf diesen Strecken gibt
		break;
case 3: backBoundarxFrontSensor = this.pose.getX()*100 - DISTANCE_FrontSideSensorToRobotCenter;
		backBoundaryFrontSensor = this.pose.getY()*100 + DISTANCE_RobotCenterToBarrier + (aktuellFrontSideSensorDistance -DISTANCE_RobotCenterToBarrier)/2;
case 7: backBoundarxFrontSensor = this.pose.getX()*100 + DISTANCE_FrontSideSensorToRobotCenter;
		backBoundaryFrontSensor = this.pose.getY()*100 - DISTANCE_RobotCenterToBarrier - (aktuellFrontSideSensorDistance -DISTANCE_RobotCenterToBarrier)/2;
		evt. chartingLogger verwenden
**/


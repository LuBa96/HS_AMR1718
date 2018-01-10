package parkingRobot;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;
import parkingRobot.INavigation.ParkingSlot;


/**
 * interface for the main module 'Navigation', providing methods for estimation of the robot position, its heading angle
 * and information about detected parking slots.
 * <p>
 * It contains the subclass {@link ParkingSlot} to provide a reasonable data structure to the cooperating classes containing
 * all relevant information for one parking slot detected by the Navigation class.
 * 
 * @author IfA
 */
public interface INavigation {
	
	// Inputs 	
	
	/**
	 * set the map of the line the robot follows while searching for parking slots
	 * 
	 * @param map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	public void setMap(Line[] map); //was mach ich konkret mit setMap?
	
	/**
	 * switches parking slot detection on or off. While a parking maneuver active parking slot detection can lead to unexpected
	 * parking slot initializations or measurements. The detection should only be activated while the robot is in an explicit
	 * parking slot search state.
	 * 
	 * @param isOn indicates if parking slot detection should be switched on (true) or off (false)
	 */
	public void setDetectionState(boolean isOn);
	
	public void setOffTrack(boolean isOn);
	
	
	
	// Class control
	
	/**
	 * initiates a new and actual computation/detection of all relevant Navigation members, especially robot pose and parking slots (last, if their detection is activated)
	 */
	public void updateNavigation();
	
	public double getfusionMatrixContent(int line, int row);
	
	public int getAktuellenKurvenpunkt();

//	public boolean getAktuellInKurve();
	
//	public void ueberpruefenObAktuellInKurve();
	
	public void PositionskorrekturAnEcken();
	

	public double getxResult();
	public double getyResult();
	public double getAngleResult();
	public double getxResultMap();
	public double getyResultMap();
	public double getAngleResultMap();
	public boolean getWinkelSchonKorrigiert();
	public double getRMS();
	
//	public void updateRobotCloseToCurve();
	public boolean getRobotCloseToCurve();
	public void setRobotCloseToCurveToFalse();
	
	public boolean backBoundaryDetektiert();
//	public boolean frontBoundaryDetektiert();
	public double backBoundarxFrontSensor();
	public double backBoundaryFrontSensor();
	public double frontBoundarxFrontSensor();
	public double frontBoundaryFrontSensor();
	public double frontBoundarxFrontSensorAktuell();
	public double frontBoundaryFrontSensorAktuell();
	public double parklueckenLaenge();
	public int anzahlParklueckenAktuelleRunde();
	public ParkingSlot getAktuellstenParkplatz();
	public ParkingSlot getParkplatz(int i);
	public int getLineNumber();
	public void setInTurn(boolean x);
	// Outputs
	
	/**
	 * returns bundled current X and Y location and heading angle phi
	 * 
	 * @return pose class containing pose at time of last computation 
	 */
	public Pose getPose();

	/**
	 * returns references to all known parking slot classes with information about ID, status, start position and end position
	 * 
	 * @return array of all known parking space class references
	 */
	public ParkingSlot[] getParkingSlots();
	
	
	// Subclasses
	
	/**
	 * represents one entry in the parking slot data bank. It contains all necessary information about one parking slot:
	 * - slot ID
	 * - slot status (whether it is/is not suitable for parking),
	 * - measured position of the slot begin front boundary, 
	 * - measured position of the slot back boundary,
	 * - quality of parking slot measurement (die Güte der Parklückenvermessung).
	 * 
	 * @author IfA
	 */
	public class ParkingSlot {		
		// members
		
		/**
		 * stores the parking slot ID. The ID is set for a new slot, added to the data bank,
		 * as the number of this new slot entry. When delegating the
		 * robot to a special parking slot a distinction between the stored slots is necessary
		 * and is given by this identification.  
		 */
		int ID;
		/**
		 * position of parking slot back boundary, robot passes it first then the front boundary
		 */
		Point backBoundaryPosition = null;
		/**
		 * position of parking slot front boundary, robot passes it second after the back boundary
		 */
		Point frontBoundaryPosition   = null;
		
		Point backBoundaryPositionM = null;
		
		Point frontBoundaryPositionM = null;
		/**
		 * defined states the parking slot can be in
		 */
		public enum ParkingSlotStatus{
			/**
			 * indicates, that the parking slot is suitable for parking
			 */
			SUITABLE_FOR_PARKING,
			/**
			 * indicates, that the parking slot is not suitable for parking
			 */
			NOT_SUITABLE_FOR_PARKING
		}
		/**
		 * characterization of the parking slot measurement with the defined state value.
		 * Per default is the parking slot not suitable.
		 */
		private ParkingSlotStatus status = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		/**
		 * stores the quality of parking slot measurement. Lower value assumes lower quality.
		 */
		int measurementQuality;
		
		// Constructor

		/**
		 * generate a ParkingSlot object without initialization of data members.
		 */
		ParkingSlot(int ID){
			this.ID = ID;
		}
		/**
		 * generate a ParkingSlot object and initialize all data members.
		 * 
		 * @param ID					 stores the parking slot ID that is the number of recent detected parking spaces incremented by one
		 * @param backBoundaryPosition	 position of parking slot back boundary, robot passes it first then the front boundary
		 * @param frontBoundaryPosition	 position of parking slot front boundary, robot passes it second after the back boundary
		 * @param slotStatus			 characterization of the parking slot measurement
		 * @param slotMeasurementQuality quality of parking slot measurement
		 * @param backBoundaryPositionM  midpoint of backBoundary for Guidance
		 * @param frontBoundaryPositionM provides information about midpoint of backBoundary for Guidance
		 */
		public ParkingSlot(int ID, Point backBoundaryPosition, Point frontBoundaryPosition, ParkingSlotStatus slotStatus, int slotMeasurementQuality, Point backBoundaryPositionM, Point frontBoundaryPositionM){
			this.ID     				= ID;
			this.backBoundaryPosition  	= backBoundaryPosition;
			this.frontBoundaryPosition	= frontBoundaryPosition;
			this.setStatus(slotStatus);		
			this.setMeasurementQuality(slotMeasurementQuality);
			this.frontBoundaryPositionM 	= frontBoundaryPositionM;
			this.backBoundaryPositionM 	= backBoundaryPositionM;
		}
		
		// Set methods

		/**
		 * @param ID stores the parking slot ID that is the number of recent detected parking spaces incremented by one
		 */
		public void setID(int ID){
			this.ID = ID;
		}
		/**
		 * @param backBoundaryPosition position of parking slot back boundary, robot passes it first then the front boundary
		 */
		public void setBackBoundaryPosition(Point backBoundaryPosition){
			this.backBoundaryPosition  = backBoundaryPosition;
		}
		/**
		 * @param frontBoundaryPosition position of parking slot front boundary, robot passes it second after the back boundary
		 */
		public void setFrontBoundaryPosition(Point frontBoundaryPosition){
			this.frontBoundaryPosition = frontBoundaryPosition;
		}
		
		public void setBackBoundaryPositionM(Point backBoundaryPositionM){
			this.backBoundaryPositionM  = backBoundaryPositionM;
		}
		
		public void setFrontBoundaryPositionM(Point frontBoundaryPositionM){
			this.frontBoundaryPositionM = frontBoundaryPositionM;
		}
		/**
		 * set parking slot status
		 * 
		 * @param status characterization of the parking slot measurement
		 */
		public void setStatus(ParkingSlotStatus status) {
			this.status = status;
		}
		/**
		 * @param slotMeasurementQuality quality of parking slot measurement
		 */
		public void setMeasurementQuality(int slotMeasurementQuality){
			this.measurementQuality = slotMeasurementQuality;
		}
		
		
		// Get methods
		
		/**
		 * @return the parking slot ID that is the number of recent detected parking spaces incremented by one
		 */
		public int getID(){
			return ID;
		}
		/**
		 * @return backBoundaryPosition position of parking slot back boundary, robot passes it first then the front boundary
		 */
		public Point getBackBoundaryPosition(){
			return backBoundaryPosition;
		}
		/**
		 * @return frontBoundaryPosition position of parking slot front boundary, robot passes it second after the back boundary
		 */
		public Point getFrontBoundaryPosition(){
			return frontBoundaryPosition;
		}
		
		public Point getBackBoundaryPositionM(){
			return backBoundaryPositionM;
		}
		
		public Point getFrontBoundaryPositionM(){
			return frontBoundaryPositionM;
		}
		/**
		 * @return status characterization of the parking slot status
		 */
		public ParkingSlotStatus getStatus() {
			return status;
		}
		/**
		 * @return quality of parking slot measurement
		 */
		public int getMeasurementQuality() {
			return measurementQuality;
		}
		
	}	
}
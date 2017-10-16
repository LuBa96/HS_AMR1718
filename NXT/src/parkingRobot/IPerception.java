package parkingRobot;

/**
 * interface for the main module 'Perception', providing methods for getting all measuring data
 * from the robot sensors
 * 
 * @author PMP
 */
public interface IPerception {
	/**
	 * returns the groundcolortype of the left linesensor
	 * use <code>{@link IPerception#calibrateLineSensors}</code> to calibrate the sensors for given environmental brightness
	 * 
	 * @return groundtype (0-white ground, 1-grey ground, 2-black ground)
	 */
	public int getLeftLineSensor();
	/**
	 * returns the groundcolortype of the right linesensor
	 * use <code>{@link IPerception#calibrateLineSensors}</code> to calibrate the sensors for given environmental brightness
	 * 
	 * @return groundtype (0-white ground, 1-grey ground, 2-black ground)
	 */	
	public int getRightLineSensor();

	/**
	 * returns the brightness of the subsurfacecolor under the left linesensor in percent 
	 * (0% for calibrated black ground; 100% for calibrated white ground).
	 * use <code>{@link IPerception#calibrateLineSensors}</code> to calibrate the sensors for given environmental brightness
	 * 
	 * @return brightness of the groundcolor in percent(%)
	 */
	public int getLeftLineSensorValue();
	
	/**
	 * returns the brightness of the subsurfacecolor under the right linesensor in percent 
	 * (0% for calibrated black ground; 100% for calibrated white ground).
	 * use <code>{@link IPerception#calibrateLineSensors}</code> to calibrate the sensors for given environmental brightness
	 * 
	 * @return brightness of the groundcolor in percent(%)
	 */
	public int getRightLineSensorValue();
	
	/**
	 * calibrate the two LineSensors 
	 * use a black and white floor color
	 * 
	 */
	public void calibrateLineSensors();	
	
	/**
	 * Displays all sensor data from the Arduino
	 *  on the LCD-Display
	 * 
	 */
	public void showSensorData();	
		
	
	/**
	 * returns the {@link EncoderSensor} Object of the right encoder for the
	 * Control-Module
	 * 
	 * @return the right Encodersensor for the <code>ControlRST</code>-Module
	 */
	public EncoderSensor getControlRightEncoder();
	/**
	 * returns the {@link EncoderSensor} Object of the left encoder for the
	 * Control-Module
	 * 
	 * @return the left Encodersensor for the <code>ControlRST</code>-Class
	 */
	public EncoderSensor getControlLeftEncoder();
	
	/**
	 * returns the Object of the {@link OdoSensor} class for the Control Module
	 * 
	 * @return OdoSensor for the <code>ControlRST</code>-Class
	 */
	public OdoSensor getControlOdo();
	
	/**
	 * returns the {@link EncoderSensor} Object of the right encoder for the
	 * Navigation-Module
	 * 
	 * @return the right Encodersensor for the <code>Navigation</code>-Module
	 */
	public EncoderSensor getNavigationRightEncoder();
	
	/**
	 * returns the {@link EncoderSensor} Object of the left encoder for the
	 * Navigation-Module
	 * 
	 * @return the left Encodersensor for the <code>Navigation</code>-Module
	 */
	public EncoderSensor getNavigationLeftEncoder();
	
	/**
	 * returns the Object of the {@link OdoSensor} class for the Navigation Module
	 * 
	 * @return OdoSensor for the <code>NavigationAT</code>-Class
	 */
	public OdoSensor getNavigationOdo();
	
	/**
	 * returns the latest measured shift in U-direction of the robot in mm.
	 * the related timedifference between the last 2 measurements returns {@link IPerception#getOdometryT}
	 * 
	 * @return U difference in mm
	 */
	public double 		getUOdmometryDiffernce();	
	/**
	 * returns the latest measured shift in V-direction of the robot in mm.
	 * the related timedifference between the last 2 measurements returns {@link IPerception#getOdometryT}
	 * 
	 * @return V difference in mm
	 */
	public double 		getVOdometryDifference();
	
	/**
	 * returns the timedifference between the last 2 Odometrymeasurements in ms
	 * 
	 * @return T time between the last 2 Odometryvalues in ms
	 */
	public int getOdometryT();
	
	/**
	 * returns the Distance of the front Sensor to the next obstacle
	 * the Value commes from the first Tri-Sensor connected to the Arduino 
	 * (the order of the Sensorports is set in the Arduino-code)
	 * 
	 * @return Distance in mm from front sensor
	 */
	public double 		getFrontSensorDistance();	
	/**
	 * returns the Distance of the front-side Sensor to the next obstacle
	 * the Value commes from the second Tri-Sensor connected to the Arduino
	 * (the order of the Sensorports is set in the Arduino-code)
	 * 
	 * @return Distance in mm from front-side sensor
	 */
	public double 		getFrontSideSensorDistance();	
	/**
	 * returns the Distance of the back Sensor to the next obstacle
	 * the Value commes from the third Tri-Sensor connected to the Arduino 
	 * (the order of the Sensorports is set in the Arduino-code)
	 * 
	 * @return Distance in mm from back sensor
	 */
	public double       getBackSensorDistance();	
	/**
	 * returns the Distance of the back-side Sensor to the next obstacle
	 * the Value commes from the fourth Tri-Sensor connected to the Arduino 
	 * (the order of the Sensorports is set in the Arduino-code)
	 * 
	 * @return Distance in mm from back-side sensor
	 */
	public double 		getBackSideSensorDistance();


	/**
	 * method for updating all the Sensorvalues
	 * by default automatically called by <code>perceptionThread</code>
	 */
	public void 		updateSensors();
	
	// Subclasses
	
	/**
	 * bundles both values relevant for measuring an angle difference by an encoder sensor
	 * 
	 * @author IfA
	 */
	public class AngleDifferenceMeasurement {
		
		/**
		 * angle that one wheel has turned since last request
		 */
		double angleSum = 0;
		
		/**
		 * time between last an actual request
		 */
		long   deltaT   = 0;
		
		/**
		 * @return returns angle that one wheel has turned since last request
		 */
		public double getAngleSum() {
			return angleSum;
		}

		/**
		 * @param angleSum sets angle that one wheel has turned since last request
		 */
		public void setAngleSum(double angleSum) {
			this.angleSum = angleSum;
		}

		/**
		 * @return returns time between last an actual request
		 */
		public long getDeltaT() {
			return deltaT;
		}

		/**
		 * @param deltaT sets time between last an actual request
		 */
		public void setDeltaT(long deltaT) {
			this.deltaT = deltaT;
		}	
	}
	
	
	/**
	 * supports processing of encoder sensor by recording the angle difference since last call of <code>getEncoderMeasurement</code>
	 * and the corresponding time interval.
	 * For every encoder information receiver pair one specific IEncoderSensor object should be instantiated.
	 * 
	 * @author IfA
	 */
	public class EncoderSensor{
		
		/**
		 * the angle that one wheel has turned since last request and the time between last an actual request. A new angle increment
		 * is added by {@link addAngle}. If this value is read by <code>getEncoderMeasurement</code> it is reset to zero.
		 */
		IPerception.AngleDifferenceMeasurement angDiffMeas = new IPerception.AngleDifferenceMeasurement();
		
		/**
		 * moment of time of last readout request
		 */
		long   ReferenceTime;
		long   CurrentTime;
		
		/**
		* Constructor of the {@link EncoderSensor} Class
		* Construction of the class starts the meausrement 
		*/
		
		
		public EncoderSensor() {
			CurrentTime   = System.currentTimeMillis();
			ReferenceTime = System.currentTimeMillis();
		}
		
		
		/**
		 * returns the angle that one wheel has turned since last request and the time between last an actual request bundled in 
		 * an AngleDifferenceMeasurement object
		 * 
		 * @return <code>AngleDifferenceMeasurement.AngleSum</code> angle sum up since last call in
		 * degrees, <code>AngleDifferenceMeasurement.deltaT</code> time delta since last call in milliseconds
		 */
		public synchronized IPerception.AngleDifferenceMeasurement getEncoderMeasurement() {
			IPerception.AngleDifferenceMeasurement ReturnAngDiffMeas = new IPerception.AngleDifferenceMeasurement();
			
			ReturnAngDiffMeas.angleSum = this.angDiffMeas.angleSum;
			ReturnAngDiffMeas.deltaT   = this.angDiffMeas.deltaT;
			
			this.angDiffMeas.angleSum = 0;
			this.angDiffMeas.deltaT   = 0;
			this.ReferenceTime        = this.CurrentTime;
			
			return ReturnAngDiffMeas;
		}

		/**
		 * adds the passed angle to the stored angle sum
		 * 
		 * @param Angle actual angle difference to be add to angle sum
		 */
		public synchronized void addAngle(double Angle) {
			this.CurrentTime 		= System.currentTimeMillis();
			angDiffMeas.angleSum 	+= Angle;
			angDiffMeas.deltaT    	= CurrentTime - ReferenceTime;		
		}
		
	}
	/**
	 * bundles all values relevant for measuring an shift difference by an Odometry sensor
	 * 
	 * @author PMP
	 */
	public class OdoDifferenceMeasurement {
		
		/**
		 * shift in x-direction since last request in mm
		 */
		double uSum = 0;
		
		/**
		 * shift in y-direction since last request in mm
		 */
		double vSum = 0;		
		/**
		 * time between last an actual request
		 */
		long   deltaT   = 0;
		
		/**
		 * @return shift in x-direction since last request in mm
		 */
		public double getUSum() {
			return uSum;
		}
		/**
		 * @return shift in y-direction since last request in mm
		 */
		public double getVSum() {
			return vSum;
		}
		
		/**
		 * @param usum sets shift difference in x-direcion since last request
		 */
		public void setUSum(double usum) {
			this.uSum = usum;
		}
		/**
		 * @param vsum sets shift difference in y-direcion since last request
		 */
		public void setVSum(double vsum) {
			this.vSum = vsum;
		}
		
		/**
		 * @return returns time between last an actual request
		 */
		public long getDeltaT() {
			return deltaT;
		}

		/**
		 * @param deltaT sets time between last an actual request
		 */
		public void setDeltaT(long deltaT) {
			this.deltaT = deltaT;
		}	
	}
	
	
	/**
	 * supports processing of the Mouse-odometrysensor by recording the Shift in U-and V-direction since last call of <code>getOdoMeasurement</code>
	 * and the corresponding time interval.
	 * For every Sensor information receiver pair one specific OdoSensor object should be instantiated.
	 * 
	 * @author PMP
	 */
	public class OdoSensor{
		
		/**
		 * the angle that one wheel has turned since last request and the time between last an actual request. A new angle increment
		 * is added by {@link addAngle}. If this value is read by <code>getOdoMeasurement</code> it is reset to zero.
		 */
		IPerception.OdoDifferenceMeasurement odoDiffMeas = new IPerception.OdoDifferenceMeasurement();
		
		/**
		 * moment of time of last readout request - not used (timeinterval from arduino is used)
		 */
		long   ReferenceTime;
		long   CurrentTime;
		
		/**
		* Constructor of the {@link OdoSensor} Class
		* Construction of the class starts the meausrement 
		*/
		public OdoSensor() {
			CurrentTime = 	System.currentTimeMillis();
			ReferenceTime = System.currentTimeMillis();
		}
		
		
		/**
		 * returns the U-,V-Shift in mm since last request and the time between last an actual request bundled in 
		 * an OdoDifferenceMeasurement object
		 * 
		 * @return <code>OdoDifferenceMeasurement.uSum</code> u-Shift since last call in
		 * mm, <code>OdoDifferenceMeasurement.vSum</code> v-Shift since last call in mm,
		 *   <code>OdoDifferenceMeasurement.deltaT</code> time delta since last call in milliseconds
		 */
		public synchronized IPerception.OdoDifferenceMeasurement getOdoMeasurement() {
			IPerception.OdoDifferenceMeasurement ReturnOdoDiffMeas = new IPerception.OdoDifferenceMeasurement();
			
			ReturnOdoDiffMeas.uSum = this.odoDiffMeas.uSum;
			ReturnOdoDiffMeas.vSum = this.odoDiffMeas.vSum;
			ReturnOdoDiffMeas.deltaT   = this.odoDiffMeas.deltaT;
			
			this.odoDiffMeas.uSum = 0;
			this.odoDiffMeas.vSum = 0;
			this.odoDiffMeas.deltaT   = 0;
			
			this.ReferenceTime        = this.CurrentTime;	//not used
			
			return ReturnOdoDiffMeas;
		}

		/**
		 * adds the passed u- and v- shift and the timedifference between the last two measurements 
		 * to the stored values
		 * 
		 * @param u			the given U-Shift in mm
		 * @param v			the given V-Shift in mm
		 * @param deltaT	the time between the last two measurements
		 */
		public synchronized void addShift(double u, double v, int deltaT) {
			this.odoDiffMeas.uSum		+= u;
			this.odoDiffMeas.vSum		+= v;
			this.odoDiffMeas.deltaT		+= deltaT;
		}
		
	}	
}

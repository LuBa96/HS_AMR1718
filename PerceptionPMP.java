package parkingRobot.hsamr0;


//import lejos.nxt.LCD;
//import java.lang.reflect.Array;
//import java.util.Arrays;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;

import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import parkingRobot.hsamr0.PerceptionThread;
import lejos.nxt.comm.*;

/**
 * Implementation of {@link IPerception}-interface.
 * 
 * ThePerception Module gets all the sensordata of the robot and provides it to the other modules
 * with respective methods
 * 
 * @author PMP
 */
public class PerceptionPMP implements IPerception {
	NXTMotor motorLeft      = null;
	NXTMotor motorRight     = null;
	IMonitor monitor = null;
	
	//!the baudrate for the RS485-connection must be the same as in the arduinocode!
	static final int RS485_BAUD = 38400;
	
	LightSensor leftLight 	= new LightSensor(SensorPort.S2);
	LightSensor rightLight 	= new LightSensor(SensorPort.S1);
	
	int RightLineSensor		=	0;
	int LeftLineSensor		=	0;
	int LSrwhite			=	0; 
	int LSrblack			=	0;
	int LSlwhite			=	0; 
	int LSlblack			=	0;


	double UOdmometry		=	0;
	double VOdometry		=	0;
	int OdometryT 			=	0; 
	
	double FrontSensorDistance		=	0;
	double FrontSideSensorDistance	=	0;
	double BackSensorDistance		=	0;
	double BackSideSensorDistance	=	0;
	
	EncoderSensor controlRightEncoder    = new EncoderSensor();
	EncoderSensor controlLeftEncoder     = new EncoderSensor();
	EncoderSensor navigationRightEncoder = new EncoderSensor();
	EncoderSensor navigationLeftEncoder  = new EncoderSensor();
	
	OdoSensor controlOdo = new OdoSensor();
	OdoSensor navigationOdo = new OdoSensor();
	
	
	byte[] readBuffer = new byte[14];
	byte[] sendBuffer = {23};
	int readBytes = 0;
	
	PerceptionThread perThread = new PerceptionThread(this);

	/**
	 * Creates a new {@code PerceptionPMP} module. 
	 * Providing methods for getting all measuring data
	 * from the robot sensors.
	 * 
	 * @param motorLeft reference to the left {@link NXTMotor}-Object
	 * @param motorRight reference to the right {@link NXTMotor}-Object
	 * @param monitor reference to main module Monitor class object
	 */
	public PerceptionPMP(NXTMotor motorLeft, NXTMotor motorRight, IMonitor monitor){
		this.motorLeft  = motorLeft;
		this.motorRight = motorRight;
		this.monitor = monitor;
		
		//build up a Connection to the Arduino via RS485
		RS485.hsEnable(RS485_BAUD,14);

		perThread.setPriority(Thread.MAX_PRIORITY - 1);
    	perThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
    	perThread.start();
    	
	}
	
	
	public synchronized int getLeftLineSensor() {
		int groundtype =1;	//standard: no black or white ground
		if(this.LeftLineSensor < (this.LSlblack+5)){	//black ground
			groundtype=2;
		}
		if(this.LeftLineSensor > (this.LSlwhite-5)){ //white ground
			groundtype=0;
		}
		return groundtype;
	}
	public synchronized int getRightLineSensor() {
		int groundtype =1;	//standard: no black or white ground
		
		if(this.RightLineSensor < (this.LSrblack+5)){	//black ground
			groundtype=2;
		}
		if(this.RightLineSensor > (this.LSrwhite-5)){ //white ground
			groundtype=0;
		}
		return groundtype;
	}
	
	public int getLeftLineSensorValue(){
		//the next line gives bin output, remove in the next version
		//return ((this.LeftLineSensor-this.LSlblack)/(this.LSlwhite-this.LSlblack))*100;
		return ((this.LeftLineSensor-this.LSlblack) *100/(this.LSlwhite-this.LSlblack));
	}
	
	public int getRightLineSensorValue(){
		//the next line gives bin output, remove in the next version
		//return ((this.RightLineSensor-this.LSrblack)/(this.LSrwhite-this.LSrblack))*100;
		return ((this.RightLineSensor-this.LSrblack) *100/(this.LSrwhite-this.LSrblack));
	}
	
	public synchronized void calibrateLineSensors(){
		LCD.clear();
		LCD.drawString("Kalibriere", 0, 0);
		LCD.drawString("Liniensensor", 0, 1);
		LCD.drawString("Weisser Untergr.", 0, 2);
		LCD.drawString("wenn position.", 0, 3);
		LCD.drawString("Enter Druecken", 0, 4);
		while(!Button.ENTER.isDown()){
			LCD.drawString("Sensorwert r:"+ (this.RightLineSensor), 0, 6);
			LCD.drawString("Sensorwert l:"+ (this.LeftLineSensor), 0, 7);
			updateSensors();
		}
		Button.ENTER.waitForPressAndRelease();
		this.LSrwhite = this.RightLineSensor;
		this.LSlwhite = this.LeftLineSensor;
		LCD.clear();
		LCD.drawString("Kalibriere", 0, 0);
		LCD.drawString("Liniensensor", 0, 1);
		LCD.drawString("Schwarzer Untergr.", 0, 2);
		LCD.drawString("wenn position.", 0, 3);
		LCD.drawString("Enter Druecken", 0, 4);
		while(!Button.ENTER.isDown()){
			LCD.drawString("Sensorwert r:"+ (this.RightLineSensor), 0, 6);
			LCD.drawString("Sensorwert l:"+ (this.LeftLineSensor), 0, 7);
			updateSensors();
		}
		Button.ENTER.waitForPressAndRelease();
		this.LSrblack = this.RightLineSensor;
		this.LSlblack = this.LeftLineSensor;
	}
	
	public void showSensorData() {
		LCD.clear();
		LCD.drawString("Sensordata ", 0, 0);
		LCD.drawString("U: "+this.UOdmometry, 0, 1);
		LCD.drawString("V: "+this.VOdometry, 0, 2);
		LCD.drawString("OdoT: "+this.OdometryT, 0, 3);
		LCD.drawString("Front: "+this.FrontSensorDistance, 0, 4);
		LCD.drawString("Fside: "+this.FrontSideSensorDistance, 0, 5);
		LCD.drawString("Back: "+this.BackSensorDistance, 0, 6);
		LCD.drawString("Bside: "+this.BackSideSensorDistance, 0, 7);
	}

	public synchronized EncoderSensor getControlRightEncoder() {
		return controlRightEncoder;
	}
	public synchronized EncoderSensor getControlLeftEncoder() {
		return controlLeftEncoder;
	}
	public synchronized OdoSensor getControlOdo() {
		return controlOdo;
	}
	public synchronized EncoderSensor getNavigationRightEncoder() {
		return navigationRightEncoder;
	}
	public synchronized EncoderSensor getNavigationLeftEncoder() {
		return navigationLeftEncoder;
	}
	public synchronized OdoSensor getNavigationOdo() {
		return navigationOdo;
	}

	
	
	public synchronized double getUOdmometryDiffernce(){
		return this.UOdmometry;
	}	
	public synchronized double getVOdometryDifference(){
		return this.VOdometry;
	}
	
	public synchronized int getOdometryT(){
		return this.OdometryT;
	}	
	
	
	
	public synchronized double getFrontSensorDistance(){		
		return this.FrontSensorDistance;
	}	
	public synchronized double getFrontSideSensorDistance(){
		return this.FrontSideSensorDistance;
	}	
	public synchronized double getBackSensorDistance(){
		return this.BackSensorDistance;
	}	
	public synchronized double getBackSideSensorDistance(){
		return this.BackSideSensorDistance;
	}
	
	
	
	public synchronized void updateSensors(){
		 updateLeftEncoderAngle();
		 updateRightEncoderAngle();
		 updateLeftLightSensor();
		 updateRightLightSensor();

		 updateArduinoSensors();
		 
		// MONITOR (example)
//		monitor.writePerceptionComment("Perception");
	}
	private void updateArduinoSensors() {
		//get all actual Measurements from Arduino
		//send 23 to get the data
		RS485.hsWrite(sendBuffer,0,sendBuffer.length);
		int readBytesSumm = 0;
		int timeoutc =0;
		byte[] sensorBytes = new byte[14];
		//wait for an answer
		while (readBytesSumm < 14 && timeoutc<20) {

			readBytes = RS485.hsRead(readBuffer, 0, readBuffer.length);
			if(readBytes>0)	//arduino sends Data
			{
				for (int i = 0; i < readBytes; i++) {
					sensorBytes[readBytesSumm+i]= readBuffer[i];
				}
				readBytesSumm+=readBytes;
			}
			timeoutc ++;
			if(timeoutc>10){	//if timeout - 2nd. try
				RS485.hsWrite(sendBuffer,0,sendBuffer.length);
				readBytesSumm=0;
			}
		}
		if(timeoutc==20) return;
		this.UOdmometry		=	(double)(((sensorBytes[1])<<8) | (sensorBytes[0] & 0xff));
		this.VOdometry		=	(double)(((sensorBytes[3])<<8) | (sensorBytes[2] & 0xff));
		this.OdometryT		=   (int)((readBuffer[5]<<8) | (readBuffer[4] & 0xff));
		this.FrontSensorDistance		=	(double)(((sensorBytes[7] & 0xff)<<8) | (sensorBytes[6] & 0xff));
		this.FrontSideSensorDistance	=	(double)(((sensorBytes[9] & 0xff)<<8) | (sensorBytes[8] & 0xff));
		this.BackSensorDistance		=		(double)(((sensorBytes[11] & 0xff)<<8) | (sensorBytes[10] & 0xff));
		this.BackSideSensorDistance	=		(double)(((sensorBytes[13] & 0xff)<<8) | (sensorBytes[12] & 0xff));		

		this.controlOdo.addShift(this.UOdmometry,this.VOdometry,this.OdometryT);
		this.navigationOdo.addShift(this.UOdmometry,this.VOdometry,this.OdometryT);
	}

	
	private void updateLeftLightSensor() {
		LeftLineSensor = leftLight.getLightValue();
	}

	private void updateRightLightSensor() {
		RightLineSensor = rightLight.getLightValue();
		
	}

	private void updateLeftEncoderAngle(){
		int deltaPhi;
		
		deltaPhi = this.motorLeft.getTachoCount();
		this.motorLeft.resetTachoCount();
		
		this.controlLeftEncoder.addAngle((double)deltaPhi);
		this.navigationLeftEncoder.addAngle((double)deltaPhi);
	}
	
	private void updateRightEncoderAngle(){
		int deltaPhi;
		
		deltaPhi = this.motorRight.getTachoCount();
		this.motorRight.resetTachoCount();
		
		this.controlRightEncoder.addAngle((double)deltaPhi);
		this.navigationRightEncoder.addAngle((double)deltaPhi);		
	}	
}




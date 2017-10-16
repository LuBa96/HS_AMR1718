package parkingRobot.hsamr0;

import java.io.IOException;
import java.util.ArrayList;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.USB;
import lejos.util.LogColumn;
import lejos.util.NXTDataLogger;
import parkingRobot.IMonitor;

/**
 * Main class for monitor module
 * 
 * @author IfA
 *
 */
public class Monitor implements IMonitor {
	
	
	// -------------------- Start: USER INPUTS --------------------
	static final monitorMode MODE = monitorMode.LOGGING_OFF; // ONLY USE "LOGGING_OFF" and "ONLINE_LOGGING"
	// --------------------- End: USER INPUTS ---------------------
	
	
	static final int TO_COLLECT = 1; // number of log entries that have to be collected AT LEAST before sending
	int collectCount = 0; // current number of collected log entries
	
	MonitorThread monThread = null;
	NXTDataLogger dlogger = new NXTDataLogger();
	boolean loggingStarted = false; // boolean that controls whether log entries can be written
	int startTime; // start time in order to calculate time stamps
	int timeStamp = 0;
	
	class StringArrayList extends ArrayList<String> {}
	class IntegerArrayList extends ArrayList<Integer> {}
	
	StringArrayList[] varNames = new StringArrayList[5]; // ArrayLists containing the names of each module's chart variables
	StringArrayList[] varValues = new StringArrayList[5]; // ArrayLists containing each module's current chart variable values
	String[] varIds = {"0;0","1;0","2;0","3;0","4;0"}; // identifiers for string format
	int[] varTimeStamps = {0,0,0,0,0}; // Array containing a time stamp for every variable
	
	StringArrayList[] comments = new StringArrayList[5]; // ArrayLists containing each module's comment strings
	String[] commentIds = {";0;1;",";1;1;",";2;1;",";3;1;",";4;1;"}; // identifiers for string format
	IntegerArrayList[] commentTimeStamps = new IntegerArrayList[5]; // Array containing a time stamp for every comment
	
	String[] outputs = {"","","","",""}; // output strings for each module (chart variable values and comments)
	String monitorOutput = ""; // output string of monitor (contains the output strings of all modules)
	
	
	/**
	 * Creates a new Monitor module
	 */
	public Monitor() {
		// check mode
		if(MODE == monitorMode.LOGGING_OFF) {
			return;
		}
		// initialize ArrayLists
		for(int i = 0; i < 5; i++) {
			varNames[i] = new StringArrayList();
			varValues[i] = new StringArrayList();
			comments[i] = new StringArrayList();
			commentTimeStamps[i] = new IntegerArrayList();
		}
		// connect if in online mode
		if(MODE == monitorMode.ONLINE_LOGGING) {
			connect();
		}
		// create and start monitor thread
		this.monThread = new MonitorThread(this);
		monThread.setPriority(Thread.MAX_PRIORITY - 2);
		monThread.setDaemon(true);
		monThread.start();
	}
	
	
	/**
	 * Establishes connection between NXTDataLogger and NXT Charting Logger; Initializes NXTDataLogger
	 */
	private void connect() {
		// user dialog
		LCD.clear();
		LCD.drawString("Monitor", 0, 0);
		LCD.drawString("Press RIGHT", 0, 2);
		LCD.drawString("for USB", 0, 3);
		LCD.drawString("Press LEFT", 0, 5);
		LCD.drawString("for Bluetooth", 0, 6);
		
		// wait until right or left button was pressed
		int buttonId = 0;
		while((buttonId != Button.ID_RIGHT) && (buttonId != Button.ID_LEFT)) {
			buttonId = Button.waitForAnyPress();
		}
		
		// user dialog
		LCD.clear();
		LCD.drawString("Monitor", 0, 0);
		LCD.drawString("Waiting...", 0, 1);
		LCD.drawString("Please open", 0, 2);
		LCD.drawString("Charting Logger", 0, 3);
		LCD.drawString("on your computer", 0, 4);
		LCD.drawString("and connect", 0, 5);
		
		// establish connection (USB or Bluetooth) according to pressed button
		NXTConnection connection = null;
		if(buttonId == Button.ID_RIGHT) {
			connection = USB.waitForConnection(30000, NXTConnection.PACKET);
		}
		else if(buttonId == Button.ID_LEFT) {
			connection = Bluetooth.waitForConnection(30000, NXTConnection.PACKET);
		}
		try {
		  dlogger.startRealtimeLog(connection);
		}
		catch (IOException e) {
		  // Do nothing.
		}
		
		// user dialog
		LCD.clear();
		LCD.drawString("Monitor", 0, 0);
		LCD.drawString("Connected!", 0, 1);
		LCD.refresh();
		Sound.beepSequenceUp();
		
		// configure log columns (This is mandatory!)
		dlogger.setColumns(new LogColumn[]{new LogColumn("Werte", LogColumn.DT_BYTE)});
	}
	
	
	public void startLogging() {
		if(MODE == monitorMode.LOGGING_OFF) {
			return;
		}
		startTime = (int) System.currentTimeMillis(); // set start time in order to calculate time stamps
		loggingStarted = true;
	}
	
	
	public void stopLogging() {
		loggingStarted = false;
		dlogger.stopLogging(); // stop NXTDataLogger
	}
	
	
	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> This method is called by the module-specific public user methods.
	 */
	private void addVar(String name, int module) {
		if(MODE == monitorMode.LOGGING_OFF) {
			return;
		}
		varNames[module].add(name);
		varValues[module].add(""); // make sure ArrayList is not empty (in order to use the set method)
	}
	
	
	public void addControlVar(String name) {
		addVar(name,0);
	}
	
	public void addGuidanceVar(String name) {
		addVar(name,1);
	}

	public void addHmiVar(String name) {
		addVar(name,2);
	}
	
	public void addNavigationVar(String name) {
		addVar(name,3);
	}
	
	public void addPerceptionVar(String name) {
		addVar(name,4);
	}
	
	
	
	/**
	 * Writes value of the specified variable to log.
	 * <p> This method is called by the module-specific public user methods.
	 */
	private void writeVar(String name, String var, int module) {
		if((!loggingStarted) || (MODE == monitorMode.LOGGING_OFF)) {
			return;
		}
		if(varNames[module].contains(name)) { // check if variable name is valid
			varValues[module].set(varNames[module].indexOf(name), var); // add variable value to the correct column
			if(!varValues[module].contains("")) { // check if all variable values were set
				varTimeStamps[module] = ((int) System.currentTimeMillis()) - startTime; // get time stamp
				collectCount++;
			}
		}
	}
	
	
	public void writeControlVar(String name, String var) {
		writeVar(name,var,0);
	}
	
	public void writeGuidanceVar(String name, String var) {
		writeVar(name,var,1);
	}
	
	public void writeHmiVar(String name, String var) {
		writeVar(name,var,2);
	}
	
	public void writeNavigationVar(String name, String var) {
		writeVar(name,var,3);
	}
	
	public void writePerceptionVar(String name, String var) {
		writeVar(name,var,4);
	}

	
	
	/**
	 * Writes comment (string) to log.
	 * <p> This method is called by the module-specific public user methods.
	 */
	private void writeComment(String str, int module) {
		if((!loggingStarted) || (MODE == monitorMode.LOGGING_OFF)) {
			return;
		}
		timeStamp = ((int) System.currentTimeMillis()) - startTime; // get time stamp
		commentTimeStamps[module].add(timeStamp);
		comments[module].add(str);
		collectCount++;
	}
	
	public void writeControlComment(String str) {
		writeComment(str,0);
	}
	
	public void writeGuidanceComment(String str) {
		writeComment(str,1);
	}
	
	public void writeHmiComment(String str) {
		writeComment(str,2);
	}
	
	public void writeNavigationComment(String str) {
		writeComment(str,3);
	}
	
	public void writePerceptionComment(String str) {
		writeComment(str,4);
	}
	
	
	
	public void run() {
		if((!loggingStarted) || (MODE == monitorMode.OFFLINE_LOGGING) || (collectCount == 0)) {
			return;
		}
		// add chart lines (variable values) to output
		for(int i = 0; i < 5; i++) {
			if(varValues[i].isEmpty() || varValues[i].contains("")) {
				continue; // continue if there are no values to send
			}
			outputs[i] = outputs[i] + "\n" + varTimeStamps[i] + ";" + varIds[i]; // output conforming to string format
			for(int j = 0; j < varValues[i].size(); j++) {
				outputs[i] = outputs[i] + ";" + varValues[i].get(j); // add the variable values to the variable output string
				varValues[i].set(j,""); // delete the added variable values
			}
		}
		// add comment (string) lines to output
		for(int i = 0; i < 5; i++) {
			while(comments[i].size() != 0) {
				outputs[i] = outputs[i] + "\n" + commentTimeStamps[i].get(0) + commentIds[i] + comments[i].get(0); // add to output conforming to string format
				comments[i].remove(0); // delete the added comments
				commentTimeStamps[i].remove(0); // delete the added time stamps
			}
		}
		// create output string
		for(int i = 0; i < 5; i++) {
			monitorOutput = monitorOutput + outputs[i];
			outputs[i] = ""; // delete the strings that were added
		}
		if(collectCount < TO_COLLECT || monitorOutput.equals("")) {
			return;
		}
		collectCount = 0; // reset counter
		dlogger.writeLog((byte) 0); // write "0" to the first and only column (This is mandatory in order to send the monitor output!)
		dlogger.writeComment(monitorOutput); // add the monitor output to the log using the comment function of NXTDataLogger
		dlogger.finishLine(); // sends logging data to NXT Charting Logger
		monitorOutput = ""; // delete string that was sent
	}

	public void sendOfflineLog() {
		/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
//		if(MODE != monitorMode.OFFLINE_LOGGING) {
//			return;
//		}
//		connect();
		 */
	}
}
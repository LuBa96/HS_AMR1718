package parkingRobot.hsamr0;

import java.io.DataInputStream;
import java.io.DataOutputStream;

import parkingRobot.IControl;
import parkingRobot.INavigation;
import parkingRobot.INxtHmi;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

/**
 * HMI server module which collects all relevant robot data and sends them to any connected client. Connection has to be p2p, so there is only one remote 
 * client per robot permitted.
 * @author PLT
 *
 */
public class HmiPLT implements INxtHmi{

	// Perception module (currently unused)
	IPerception perception;
	// Navigation module delivers robot position and parking slot information
	INavigation navigation;
	// Control module (currently unused)
	IControl control;
	// Monitor module
	IMonitor monitor;
	
	// Changed from original version!
//	// separate Thread running the input operations of this module. 
//	HmiReaderThread hmiReaderThread = new HmiReaderThread(this);
//	
//	// separate Thread running the output operations of this module. 
//	HmiSenderThread hmiSenderThread = new HmiSenderThread(this);
	
	protected int selectedParkingSlot;
	int noOfParkingSlots = 0;
	protected Mode mode;
	private BTConnection connection;
	protected DataInputStream dataIn;
	DataOutputStream dataOut;
	
	// Whether to control the robot via HMI or directly by pressing buttons on the device
	protected boolean useHMI;
	
	/**
	 * Check whether the HMI on the Android device is used at all
	 * @return Whether the HMI on the Android device is used
	 */
	public boolean isUseHMI() {
		return useHMI;
	}

	/**
	 * Enumeration holding all possible message codes. Incoming requests have prefix IN_ and outgoing data have prefix OUT_, respectively.
	 * @author PLT
	 *
	 */
	public enum Command {
		/**
		 * Command to identify a message that contains the chosen mode
		 */
		IN_SET_MODE,
		
		/**
		 * Command to identify a message that contains the chosen parking slot
		 */
		IN_SELECTED_PARKING_SLOT,

		/**
		 * Command to identify a message that contains the current position
		 */
		OUT_POSITION,
		
		/**
		 * Command to identify a message that contains the parking slot to park in
		 */
		OUT_PARKSLOT,
		
		/**
		 * Command to identify a message that contains the current STATUS
		 */
		OUT_STATUS
	}

	/**
	 * Creates a new {@code HmiPlt} module. It accesses position and parking slot information from provided modules for navigation, perception and 
	 * control.
	 * @param perception reference to instantiated perception module
	 * @param navigation reference to instantiated navigation module
	 * @param control reference to instantiated control module
	 * @param monitor reference to instantiated monitor module
	 */
	public HmiPLT (IPerception perception, INavigation navigation, IControl control, IMonitor monitor) {
		this.perception = perception;
		this.navigation = navigation;
		this.control = control;
		this.monitor = monitor;
		
		// Changed from original version!
		// separate Thread running the input operations of this module. 
		HmiReaderThread hmiReaderThread = new HmiReaderThread(this, monitor);
		// separate Thread running the output operations of this module. 
		HmiSenderThread hmiSenderThread = new HmiSenderThread(this, monitor);
		
		// Start connect mode (waiting to receive connection request from remote)
		connect();
		
		// Start the HMI reader and sender threads
		hmiReaderThread.setName("ReaderThread");
		hmiReaderThread.setPriority(Thread.MAX_PRIORITY -1);
		hmiReaderThread.setDaemon(false);
		hmiReaderThread.start();
		
		hmiSenderThread.setName("SenderThread");
		hmiSenderThread.setPriority(Thread.MAX_PRIORITY -1);
		hmiSenderThread.setDaemon(false);
		hmiSenderThread.start();
	
	}
	


	@Override
	public int getSelectedParkingSlot() {
		return selectedParkingSlot;
	}

	@Override
	public Mode getMode() {
		return mode;
	}
	
	/**
	 * Sets the robot to connect mode, which is passive. It waits for an incoming connection request. This method will not return until connection is established or a built-in connection 
	 * timeout was raised.
	 */
	protected boolean connect(){
		LCD.clear();
	    LCD.drawString("Bluetooth:", 0, 0);
	    LCD.drawString("Connect -> Enter", 0, 1);
	    LCD.drawString("Don't -> Esc", 0, 2);
		LCD.refresh();
		
		while(true){
			if ( Button.ENTER.isDown() ){
				while(Button.ENTER.isDown()){try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}} //wait for button release
				
				LCD.clear();
			    LCD.drawString("Waiting for", 0, 0);
			    LCD.drawString("Bluetooth...", 0, 1);
				LCD.refresh();

		        connection = Bluetooth.waitForConnection(); // this method is very patient.

			    // Get I/O streams from BT connection
			    dataIn = connection.openDataInputStream();
			    dataOut = connection.openDataOutputStream();
			    if(dataIn == null || dataOut == null)
			    {
			    	// Connection failed
					LCD.clear();
				    LCD.drawString("Connection failed", 0, 0);
					LCD.refresh();
					Sound.beepSequence();
					useHMI = false;
					return false;
			    }
			    else
			    {
			    	// Connection successful
					LCD.clear();
					LCD.drawString("Connected",0,0);
					LCD.refresh();	
			    	Sound.beepSequenceUp();
			    	useHMI = true;
			    	return true;
			    }
			}else if ( Button.ESCAPE.isDown() ){
				while(Button.ESCAPE.isDown()){try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}} //wait for button release
				useHMI = false;
				return false;
			}
		}
		
		
	    
	}
	
	/**
	 * Closes the connection to remote control device.
	 */
	public void disconnect() {
		if(useHMI){
			connection.close();
			dataIn = null;
			dataOut = null;
		}		
	}

}

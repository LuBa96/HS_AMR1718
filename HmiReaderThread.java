package parkingRobot.hsamr0;

import java.io.IOException;

import parkingRobot.IMonitor;
import parkingRobot.INxtHmi.Mode;
import parkingRobot.hsamr0.HmiPLT.Command;

/**
 * Thread for remote communication via Bluetooth. Hosts the HMI module of class {@code HmiPlt}, from which it was started. For 
 * simplification each main module class has its own calculation thread calculate all relevant algorithms independent from the 
 * other modules. In case of shared data access with other main module classes synchronized access must be guaranteed.
 * @author PLT
 *
 */
public class HmiReaderThread extends Thread{

	HmiPLT hmi;
	IMonitor monitor;

	/**
	 * The HmiThread constructor gets the hmi object
	 * @param hmi hmi object reference
	 * @param monitor monitor object reference 
	 */
	public HmiReaderThread(HmiPLT hmi, IMonitor monitor) {
		this.hmi = hmi;
		this.monitor = monitor;
	}

	@Override
	public void run() {
		while(true){			 
			// If we us the HMI at all process inputs
			if(hmi.useHMI){
				// First process commands from remote control, which is more important than sending data for visualization.
				processInputs();
				
				// MONITOR (example)
//				monitor.writeHmiComment("Hmi_Reader");
			}

			try{	            	
				Thread.sleep(100);

			} catch(InterruptedException ie){
				System.out.println("Interruption of HmiReaderThread in sleep()");
			}
		}
	}

	/**
	 * Reads current input messages to change driving mode or selected parking slot.
	 * This methods is to be called from the {@link HmiReaderThread HmiThread}. Execution time should be almost equal, because each message consists of 
	 * two integer data. 
	 */
	private synchronized void processInputs() {
		int code;
		try
		{
			// Careful: the read*() methods are blocking!
			code = hmi.dataIn.readInt();
			Command command = Command.values()[code];
			if (command == Command.IN_SET_MODE) {
				int imode = hmi.dataIn.readInt();
				hmi.mode = Mode.values()[imode];
			} else if (command == Command.IN_SELECTED_PARKING_SLOT) {
				hmi.selectedParkingSlot = hmi.dataIn.readInt();
			}
		} catch (IOException e)
		{
			System.out.println("Read exception "+e);
		}
	}

}

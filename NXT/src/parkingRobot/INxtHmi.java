package parkingRobot;

/**
 * Robot-side human machine interface module. This part is acting as a server. It periodically sends status and position information to the connected 
 * remote client and receives requests for changing mode or selected parking slots.
 * @author PLT
 *
 */
public interface INxtHmi {

	/**
	 * Enumeration holding the robot driving modes.
	 * @author PLT
	 *
	 */
	public enum Mode {
		/**
		 * Scouting for parking spots
		 */
		SCOUT,
		
		/**
		 * Find the next free parking slot and park
		 */
		PARK_NOW,
		
		/**
		 * Park in a selected slot
		 */
		PARK_THIS,
		
		/**
		 * Have a coffee
		 */
		PAUSE,
		
		/**
		 * Close bluetooth connection 
		 */
		DISCONNECT;
	}
	
	/**
	 * Closes the connection to remote control device.
	 */
	public void disconnect();
	
	/**
	 * Retrieves the currently selected parking slot to be used by the other robot modules. The parking slot is set via user request from a remote device.
	 * @return ID of the currently selected parking slot
	 */
	public int getSelectedParkingSlot();
	
	/**
	 * Returns the current driving mode, which is a value of {@link Mode Mode}. The driving mode is set via user request from a remote device.
	 * @return the current driving mode
	 */
	public Mode getMode();
}


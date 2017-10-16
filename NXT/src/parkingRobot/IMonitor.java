package parkingRobot;

/**
 * Interface for the module "Monitor"
 * <p>
 * Provides methods for advanced monitoring and debugging
 * 
 * @author IfA
 *
 */
public interface IMonitor {

	/**
	 * Monitor starts logging.
	 */
	public void startLogging();

	/**
	 * Monitor stops logging and disconnects.
	 */
	public void stopLogging();
	
	/**
	 * Three modes for monitoring: Online-logging, offline-logging, logging turned off
	 */
	public enum monitorMode {
		/**
		 * Sends log continuously while program is running 
		 */
		ONLINE_LOGGING,
		
		/**
		 * Sends log when program is exited
		 * <p> DO NOT USE (NOT IMPLEMENTED)
		 */
		OFFLINE_LOGGING,
		
		/**
		 * Logging turned off
		 */
		LOGGING_OFF
	}
	
	

	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> The variables will be written to the log in the same order as they were added.
	 * <p> Use ONLY for control module!
	 * @param name Add the name to identify the monitored variable.
	 */
	public void addControlVar(String name);
	
	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> The variables will be written to the log in the same order as they were added.
	 * <p> Use ONLY for guidance module!
	 * @param name Add the name to identify the monitored variable.
	 */
	public void addGuidanceVar(String name);
	
	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> The variables will be written to the log in the same order as they were added.
	 * <p> Use ONLY for hmi module!
	 * @param name Add the name to identify the monitored variable.
	 */
	public void addHmiVar(String name);
	
	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> The variables will be written to the log in the same order as they were added.
	 * <p> Use ONLY for navigation module!
	 * @param name Add the name to identify the monitored variable.
	 */
	public void addNavigationVar(String name);
	
	/**
	 * Creates a new spot in the log to monitor a variable.
	 * <p> The variables will be written to the log in the same order as they were added.
	 * <p> Use ONLY for perception module!
	 * @param name Add the name to identify the monitored variable.
	 */
	public void addPerceptionVar(String name);
	
	
	
	/**
	 * Writes value of the specified variable to log.
	 * <p> Add the name of the variable using "addControlVar" BEFORE using this method
	 * <p> Method should only be called once per variable. The order in which variables are written does not matter.
	 * <p> Use ONLY for control module!
	 * @param name Add the name to identify the monitored variable.
	 * @param var Value of the variable to be monitored (cast to string).
	 */
	public void writeControlVar(String name, String var);
	
	/**
	 * Writes value of the specified variable to log.
	 * * <p> Add the name of the variable using "addGuidanceVar" BEFORE using this method
	 * <p> Method should only be called once per variable. The order in which variables are written does not matter.
	 * <p> Use ONLY for guidance module!
	 * @param name Add the name to identify the monitored variable.
	 * @param var Value of the variable to be monitored (cast to string).
	 */
	public void writeGuidanceVar(String name, String var);
	
	/**
	 * Writes value of the specified variable to log.
	 * * <p> Add the name of the variable using "addHmiVar" BEFORE using this method
	 * <p> Method should only be called once per variable. The order in which variables are written does not matter.
	 * <p> Use ONLY for hmi module!
	 * @param name Add the name to identify the monitored variable.
	 * @param var Value of the variable to be monitored (cast to string).
	 */
	public void writeHmiVar(String name, String var);
	
	/**
	 * Writes value of the specified variable to log.
	 * * <p> Add the name of the variable using "addNavigationVar" BEFORE using this method
	 * <p> Method should only be called once per variable. The order in which variables are written does not matter.
	 * <p> Use ONLY for navigation module!
	 * @param name Add the name to identify the monitored variable.
	 * @param var Value of the variable to be monitored (cast to string).
	 */
	public void writeNavigationVar(String name, String var);
	
	/**
	 * Writes value of the specified variable to log.
	 * * <p> Add the name of the variable using "addPerceptionVar" BEFORE using this method
	 * <p> Method should only be called once per variable. The order in which variables are written does not matter.
	 * <p> Use ONLY for perception module!
	 * @param name Add the name to identify the monitored variable.
	 * @param var Value of the variable to be monitored (cast to string).
	 */
	public void writePerceptionVar(String name, String var);
	
	
	
	/**
	 * Writes comment (string) to log.
	 * <p> Use ONLY for control module!
	 * @param str Add the string to be logged.
	 */
	public void writeControlComment(String str);
	
	/**
	 * Writes comment (string) to log.
	 * <p> Use ONLY for guidance module!
	 * @param str Add the string to be logged.
	 */
	public void writeGuidanceComment(String str);
	
	/**
	 * Writes comment (string) to log.
	 * <p> Use ONLY for hmi module!
	 * @param str Add the string to be logged.
	 */
	public void writeHmiComment(String str);
	
	/**
	 * Writes comment (string) to log.
	 * <p> Use ONLY for navigation module!
	 * @param str Add the string to be logged.
	 */
	public void writeNavigationComment(String str);
	
	/**
	 * Writes comment (string) to log.
	 * <p> Use ONLY for perception module!
	 * @param str Add the string to be logged.
	 */
	public void writePerceptionComment(String str);
	
	
	
	/**
	 * Main method of monitor module
	 */
	public void run();
	
	/**
	 * Sends log if monitor is in offline mode.
	 */
	public void sendOfflineLog();
}

package parkingRobot.hsamr0;

import parkingRobot.INavigation;

/**
 * thread started by the 'Navigation' class for background calculating. For simplification each main module class has its own
 * calculation thread calculate all relevant algorithms independent from the other modules. In case of shared data access with
 * other main module classes synchronized access must be guaranteed.
 * 
 * @author IfA
 */
public class NavigationThread extends Thread {
    
	/**
	 * corresponding main module Navigation class object
	 */
	INavigation navigation;
	
	/**
	 * provides the reference transfer so that the navigation thread knows its corresponding navigation object 
	 * 
	 * @param navigation corresponding main module Navigation class object
	 */
	NavigationThread(INavigation navigation){
    	this.navigation = navigation;
    }
	
	/* (non-Javadoc)
	 * @see java.lang.Thread#run()
	 */
	@Override
    public void run() {
        while(true){
        	try{
        		// Attention: When the program grows, a good synchronization concept is necessary to avoid
        		// inconsistency or corrupted data. In this example the whole updateNavigation method is
        		// synchronized. If more than one methods are called a synchronized block around all methods
        		// might be necessary!
            	navigation.updateNavigation();
	            
            	// A good seep time trade off is necessary:
            	// To less limit the CPU time of other threads,
            	// to much makes the increment errors to big and the navigation slow            	
            	Thread.sleep(100);
            	
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	
}
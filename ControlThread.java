package parkingRobot.hsamr0;

import parkingRobot.IControl;

/**
 * thread started by the 'Control' class for background calculating. For simplification each main module class has its own
 * calculation thread calculate all relevant algorithms independent from the other modules. In case of shared data access with
 * other main module classes synchronized access must be guaranteed.
 * 
 */
public class ControlThread extends Thread {
    
	/**
	 * 
	 */
	IControl control;
	
	
	/**
	 * 
	 * 
	 * @param perception
	 */
	ControlThread(IControl control){
    	this.control = control;
    }
	
	
	/* (non-Javadoc)
	 * @see java.lang.Thread#run()
	 */
	@Override
    public void run() {
        while(true){
        	try{
            	control.exec_CTRL_ALGO();
	            Thread.sleep(100);
        	} catch(InterruptedException ie){	        		
        	}
        }
    }	

}

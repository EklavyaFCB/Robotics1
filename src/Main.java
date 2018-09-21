package A1;

import lejos.hardware.Button;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class Main {

	public static void main(String[] args) {
		
		PilotRobot me = new PilotRobot();		
		PilotMonitor myMonitor = new PilotMonitor(me, 400);	
		//GridDisplay myMonitor = new GridDisplay(me, 400);	

		// Set up the behaviours for the Arbitrator and construct it.
		//Behavior b1 = new DriveForward(me);
		//Behavior b2 = new BackUp(me);
		Behavior b3 = new GoAround(me);
		Behavior b4 = new SearchAllGrid(me);
		Behavior [] bArray = {b3,b4};
		Arbitrator arby = new Arbitrator(bArray);

		// Note that in the Arbritrator constructor, a message is sent
		// to stdout.  The following prints eight black lines to clear
		// the message from the screen
     	for (int i=0; i<8; i++)
     	System.out.println("");

     	// Start the Pilot Monitor
		myMonitor.start();

		// Tell the user to start
		myMonitor.setMessage("Press a key to start");				
     	Button.waitForAnyPress();
     
     	// Start the Arbitrator
		arby.go();
	}

}
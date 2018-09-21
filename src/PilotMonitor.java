package A1;

//PillotMonitor.java
//
//Based on the RobotMonitor class, this displays the robot
//state on the LCD screen; however, it works with the PilotRobot
//class that exploits a MovePilot to control the Robot.
//
//Terry Payne
//8th October 2017
//

import java.text.DecimalFormat;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;

public class PilotMonitor extends Thread {

	private int delay;
	public PilotRobot robot;
	private String msg;
	
 GraphicsLCD lcd = LocalEV3.get().getGraphicsLCD();
	
 // Make the monitor a daemon and set
 // the robot it monitors and the delay
 public PilotMonitor(PilotRobot r, int d){
 	this.setDaemon(true);
 	delay = d;
 	robot = r;
 	msg = "";
 }
 
 // Allow extra messages to be displayed
 public void resetMessage() {
 	this.setMessage("");
 }
 
 // Clear the message that is displayed
 public void setMessage(String str) {
 	msg = str;
 }

 // The monitor writes various bits of robot state to the screen, then
 // sleeps.
 public void run(){
 	// The decimalformat here is used to round the number to three significant digits
		DecimalFormat df = new DecimalFormat("####0.000");
		boolean type =true;
 	while(true){
 		int temp_type = Button.waitForAnyPress(500);
 		type = temp_type==Button.ID_RIGHT?!type:type;
 		lcd.clear();	

 		if(!type) {
 			robot.updateProbability();
 			for(int i=0;i<robot.GRID_LNUMBER;i++) {
	 			for(int j=0;j<robot.GRID_WNUMBER;j++) {
	 				lcd.drawString(String.format("%.1f", new Double(robot.array_prob[i][j]*100).isNaN()?0:robot.array_prob[i][j]*100), j*38, i*20, GraphicsLCD.VCENTER);
	 				
	 			}
	 		}
 		}
 		else if(type){
	 		for(int i=0;i<robot.GRID_LNUMBER+2;i++) {
	 			for(int j=0;j<robot.GRID_WNUMBER+2;j++) {
	 				lcd.drawRect(j*15,i*15, 15, 15);
	 				if(i==0||j==0|| i==robot.GRID_LNUMBER+1 || j==robot.GRID_WNUMBER+1)
	 					lcd.fillRect(j*15,i*15 , 15, 15);
	 				else {
		 				if(robot.array_M[i-1][j-1]>0)
		 					lcd.fillRect(j*15,i*15 , 15, 15);
		 				if((i-1)==robot.currentX && (j-1)==robot.currentY)
		 					lcd.drawChar('X', j*15, i*15, GraphicsLCD.VCENTER);
		 				if(robot.array_been[i-1][j-1]==0)
		 					lcd.drawChar('n', j*15, i*15, GraphicsLCD.VCENTER);
	 				}
	 			}
	 		}
 		}
 		
 		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
 		//lcd.drawString("UT heading"+robot.getUpMotor(), 0, 160, 0);
 		
	    }
	   
 }

}
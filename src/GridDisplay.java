// This class is being used to show the occupancy grid on the LCD screen
// Currently for testing - implemented inside another class if required later on.
package A1;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;


public class GridDisplay extends Thread {

	// Main
	public static void main(String[] args) {
		
		// Declarations
		final int WALL_LENGTH = 8;
		final int WALL_WIDTH = 8;
		int[][] array = new int[WALL_WIDTH][WALL_LENGTH];
		// Declarations
		// Get the default instance of the Brick, to get access to its resources
	
		GraphicsLCD lcd = LocalEV3.get().getGraphicsLCD();

		// DISPLAY
        LCD.clear();
        
        // Boundary 
        lcd.drawRect(0, 0, lcd.getWidth(), lcd.getHeight());
        
 		// Grids
 		for (int i=0; i<WALL_WIDTH; i++) {
 			for (int j=0; j<WALL_LENGTH; j++) {
 				//Creating the outside walls
 				if (i == 0 || j == 0 || i == 7 || j == 7) {
 	 				array[i][j] = 1;
 	 			}
 				else if (i==2 && j==4 || i==5 && j==3 || i==3 && j==2 || i==3  && j==5) {
 					array[i][j] = 1;
 				}
 				//Placing the paper/end point (not needed for final implementation)
 				else if(i==3 && j==4) {
 					array[i][j] = 2;
 				}
 				//Setting found path (not needed for final implementation)
 				else if ((i==1 && j==2) || (i==1 && j==3) || (i==2 && j==3) || (i==3 && j==3)) {
 					array[i][j] = 4;
 				}
 				//Setting the position of the robot
 				else if (i==1 && j==1) {
 					array[i][j] = 5;
 				}
 				//Cells that are found empty (will be 0 until scanned)
 				else {
 					array[i][j] = 3;
 				}
 				//Drawing the grid with array
 				lcd.drawRect(i*lcd.getWidth()/WALL_WIDTH,j*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
 				lcd.drawString(Integer.toString(array[i][j]),(i*lcd.getWidth()/WALL_WIDTH)+((lcd.getWidth()/WALL_WIDTH)/3),j*lcd.getHeight()/WALL_LENGTH+2,0);
 			}
 		}
 		
 		// Update cells - needs editing with final implementation (currently turns all cells to obstacles
		if (!Button.ESCAPE.isDown()) {
	 		for (int i=0; i<WALL_WIDTH; i++) {
	 			for (int j=0; j<WALL_LENGTH; j++) {
	 				lcd.drawRect(i*lcd.getWidth()/WALL_WIDTH,j*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
					// We want the string to be drawn in the 'middle' of each grid, so we add a third of the width of a grid to the starting x coordinate of the string
	 				/*lcd.clear();
	 				array[i][j] = 1;
	 				lcd.drawRect(i*lcd.getWidth()/WALL_WIDTH,j*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
	 				lcd.drawString(Integer.toString(array[i][j]),(i*lcd.getWidth()/WALL_WIDTH)+((lcd.getWidth()/WALL_WIDTH)/3),j*lcd.getHeight()/WALL_LENGTH+2,0);
	 				for (int m=0; m<WALL_WIDTH; m++) {
	 		 			for (int n=0; n<WALL_LENGTH; n++) {
	 		 				if (i==m && j==n) {
	 		 					
	 		 				}
	 		 				else {
	 		 				lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
	 		 				lcd.drawString(Integer.toString(array[m][n]),(m*lcd.getWidth()/WALL_WIDTH)+((lcd.getWidth()/WALL_WIDTH)/3),n*lcd.getHeight()/WALL_LENGTH,0);
	 		 				}
	 		 			}
	 				}
	 		 		try{
	 		 			sleep(100);
	 		 		}
	 		 		catch(Exception e){
	 		 			// We have no exception handling
	 		 		}*/
	 			}
	 		}
		} 
		else {
	        lcd.clear();
		}
		
		//Changing display to graphic grid
		while (!Button.ESCAPE.isDown()) {
			if (Button.RIGHT.isDown()) {
		 		for (int i=0; i<WALL_WIDTH; i++) {
		 			for (int j=0; j<WALL_LENGTH; j++) {
						// We want the string to be drawn in the 'middle' of each grid, so we add a third of the width of a grid to the starting x coordinate of the string
		 				lcd.clear();
		 				lcd.drawRect(i*lcd.getWidth()/WALL_WIDTH,j*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 				
		 				for (int m=0; m<WALL_WIDTH; m++) {
		 		 			for (int n=0; n<WALL_LENGTH; n++) {
		 		 				//Drawing the obstacles
		 		 				if (array[m][n] == 1) {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 					lcd.fillRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 				}
		 		 				//Drawing the paper
		 		 				else if (array[m][n] == 2) {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 					//Drawing plus and cross
		 		 					/*lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH+11,n*lcd.getHeight()/WALL_LENGTH,m*lcd.getWidth()/WALL_WIDTH+11,n*lcd.getHeight()/WALL_LENGTH+16);
		 		 					lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH+8,m*lcd.getWidth()/WALL_WIDTH+22,n*lcd.getHeight()/WALL_LENGTH+8);
		 		 					lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,m*lcd.getWidth()/WALL_WIDTH+22,n*lcd.getHeight()/WALL_LENGTH+16);
		 		 					lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH+22,n*lcd.getHeight()/WALL_LENGTH,m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH+16);*/
		 		 					//Drawing rectangles
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH+4,n*lcd.getHeight()/WALL_LENGTH+4,lcd.getWidth()/WALL_WIDTH-8,lcd.getHeight()/WALL_LENGTH-8);
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH+6,n*lcd.getHeight()/WALL_LENGTH+6,lcd.getWidth()/WALL_WIDTH-12,lcd.getHeight()/WALL_LENGTH-12);
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH+8,n*lcd.getHeight()/WALL_LENGTH+8,lcd.getWidth()/WALL_WIDTH-16,lcd.getHeight()/WALL_LENGTH-16);
		 		 				}
		 		 				//Drawing the path
		 		 				else if (array[m][n] == 4) {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 					//Drawing cross over path
		 		 					lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,m*lcd.getWidth()/WALL_WIDTH+22,n*lcd.getHeight()/WALL_LENGTH+16);
		 		 					lcd.drawLine(m*lcd.getWidth()/WALL_WIDTH+22,n*lcd.getHeight()/WALL_LENGTH,m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH+16); 
		 		 				}
		 		 				//Drawing robot location
		 		 				else if (array[m][n] == 5) {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 					//Drawing circle for robot
		 		 					lcd.fillArc(m*lcd.getWidth()/WALL_WIDTH+4,n*lcd.getHeight()/WALL_LENGTH+1,14,14,n*lcd.getWidth()/WALL_LENGTH+4,360);
		 		 				}
		 		 				//Everything else is empty
		 		 				else {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 				}
		 		 			}
		 				}
		 			}
		 		}
			}
			
			//Changing display to text grid
		 	else if (Button.LEFT.isDown()) {
		 		for (int i=0; i<WALL_WIDTH; i++) {
		 			for (int j=0; j<WALL_LENGTH; j++) {
		 				lcd.clear();
		 				for (int m=0; m<WALL_WIDTH; m++) {
		 		 			for (int n=0; n<WALL_LENGTH; n++) {
		 		 				//Redrawing numbers
		 		 				if (array[m][n] == 0 || array[m][n] == 1 || array[m][n] == 2 || array[m][n] == 3 || array[m][n] == 4 || array[m][n] == 5 ) {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
			 		 				lcd.drawString(Integer.toString(array[m][n]),(m*lcd.getWidth()/WALL_WIDTH)+((lcd.getWidth()/WALL_WIDTH)/3),n*lcd.getHeight()/WALL_LENGTH+2,0);
		 		 				}
		 		 				//Anything else will appear empty (error check)
		 		 				else {
		 		 					lcd.drawRect(m*lcd.getWidth()/WALL_WIDTH,n*lcd.getHeight()/WALL_LENGTH,lcd.getWidth()/WALL_WIDTH,lcd.getHeight()/WALL_LENGTH);
		 		 				}
		 		 			}
		 				}
		 			}
		 		}
			} 
		 	else {}
		}
	}
}
package A1;

import java.util.List;

import lejos.hardware.Button;
import lejos.robotics.subsumption.Behavior;

public class SearchAllGrid implements Behavior {

	public boolean suppressed;
	private PilotRobot me;
	private int nextX = 0;													//the next point X
	private int nextY = 0;													//the next point Y
	private List shorestPath;												//the next point path list

	// Constructor - store a reference to the robot
	public SearchAllGrid(PilotRobot robot){
		me = robot;
 	}
	
	@Override
	public boolean takeControl() {
		return me.go_around_finished;										//if stop the go around action, then take control
	}

	@Override
	public void action() {
		nextPoint();															//calculate the next point
		//System.out.println("start:"+nextX+" "+nextY);
		//System.out.println(shorestPath);
		for(int i=0;i<shorestPath.size();i=i+2) {
			//Button.waitForAnyPress();				//for debug mode
//			if(me.getColour()[0]<0.01){
//				me.getPilot().stop();
//				Button.waitForAnyPress();
//				System.exit(0);
//			}
			detect();														//collect the data
			Goto((int)shorestPath.get(i),(int)shorestPath.get(i+1));			//do the action to the next point
			if(me.go_around_finished && me.end_X>=0 && me.end_Y>=0 && me.currentX==me.end_X && me.currentY==me.end_Y)
			{																//if the end point is found, then stop
				me.getPilot().stop();
				int type = 0;
				type=Button.waitForAnyPress();
				while(type!=Button.ID_ESCAPE)
					type=Button.waitForAnyPress();
				System.exit(0);
			}
		}
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		suppressed = true;
	}
	
	public void Goto(int passX,int passY) {													//go to next X,Y point
		//System.out.println("pass:"+passX+" "+passY);
		float rotate_angle = me.getPose().getHeading();
		rotate_angle = Math.round(rotate_angle/90.0)*90;
		double rotate_radian = rotate_angle*Math.PI/180;
		int currentX_heading = (int) (Math.round(100*Math.cos(rotate_radian))/100);
		int currentY_heading = (int) (Math.round(100*Math.sin(rotate_radian))/100);			//calculate the current heading
		int currentX = me.currentX;															//get the current position
		int currentY = me.currentY;
		//System.out.println(currentX+" "+currentY);
		//System.out.println(currentX_heading+" "+currentY_heading);							//use formula to do the action
		if((currentX_heading==(passX-currentX)&&currentX_heading!=0) || (currentY_heading==(passY-currentY) && currentY_heading!=0))
			me.goNeighbour(1);
		if((-currentX_heading==(passX-currentX)&&currentX_heading!=0) || (-currentY_heading==(passY-currentY)&&currentY_heading!=0))
			me.goNeighbour(2);
		if((currentY_heading==(passX-currentX)&&currentY_heading!=0)||(currentX_heading==-(passY-currentY) && currentX_heading!=0))
			me.goNeighbour(3);
		if((currentY_heading==-(passX-currentX)&&currentY_heading!=0) || (currentX_heading==(passY-currentY)&&currentX_heading!=0))
			me.goNeighbour(4);
	}
	
	public void nextPoint() {																//find the closest point that have not been there yet
		me.updateFinal();																	//first update the final matrix
		if(me.go_around_finished && me.end_X>=0 && me.end_Y>=0)
		{
			me.array_final[me.end_X][me.end_Y]=2;											//set color paper as dest
			AStarSearch grid = new AStarSearch(me.array_final.length,me.array_final[0].length,me.array_final);		//use A star method
			grid.getDestination(); 
			List path = grid.aSearch(me.currentX,me.currentY);								//then find the path list
			nextX = me.end_X;
			nextY = me.end_Y;
			shorestPath = path;																//update the path
			return;
		}
		int shortestSize=100000;																//initial the size to large number
		for(int i=0;i<me.GRID_LNUMBER;i++) {
			for(int j=0;j<me.GRID_WNUMBER;j++) {
				if(me.array_been[i][j]==0&&me.array_final[i][j]==0) {							//if the point is the not been yet and no obstacle
					me.array_final[i][j]=2;													//then set it as the dest
					AStarSearch grid = new AStarSearch(me.array_final.length,me.array_final[0].length,me.array_final);
					grid.getDestination(); 
					List path = grid.aSearch(me.currentX,me.currentY);						//produce the path list
					
					if(path.size()<shortestSize)												//compare to get the shorest path
					{
						shortestSize = path.size();
						nextX = i;
						nextY = j;
						shorestPath = path;
					}
				}
			}
		}
	}
	
	public void detect() {																//the detect method, the same as the GoAround class function
		float distance_forward = me.getDistance();
		me.turnUMLeft();
		float distance_left = me.getDistance();
		me.turnUMRight();
		float distance_right = me.getDistance();
		me.turnUMForward();
		float rotate_angle = me.getPose().getHeading();
		rotate_angle = Math.round(rotate_angle / 90.0) * 90;
		double rotate_radian = rotate_angle * Math.PI / 180;

		float Forward_x = distance_forward * (float) Math.cos(rotate_radian);
		float Forward_y = distance_forward * (float) Math.sin(rotate_radian);

		float Left_x = distance_left * (float) Math.sin(rotate_radian);
		float Left_y = -distance_left * (float) Math.cos(rotate_radian);

		float Right_x = -distance_right * (float) Math.sin(rotate_radian);
		float Right_y = distance_right * (float) Math.cos(rotate_radian);

		me.updateMatrix(Forward_x, Forward_y);
		me.updateMatrix(Left_x, Left_y);
		me.updateMatrix(Right_x, Right_y);
	}

}

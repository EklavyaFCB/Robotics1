package A1;

import lejos.hardware.Button;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Behavior;

public class GoAround implements Behavior {

	public boolean suppressed;													//for supressed
	private PilotRobot me;
	private int count;															//for count the time running
	private static boolean obstacleForward = false;								//boolean flag whether there is a obstacle forward
	private static boolean obstacleLeft = true;									//boolean flag whether there is a obstacle left
	private static boolean obstacleRight = false;								//boolean flag whether there is a obstacle right

	// Constructor - store a reference to the robot
	public GoAround(PilotRobot robot) {
		me = robot;
		count = 1;
	}

	@Override
	public boolean takeControl() { 												//take the control until the back to the start point
		return !me.go_around_finished;
	}

	@Override
	public void action() {
		// Allow this method to run
		suppressed = false;
		//detect();
		while (!suppressed) {
			//Button.waitForAnyPress();				//for debug mode
			if(me.getColour()[0]<0.01){											//for debug mode
				me.getPilot().stop();
				Button.waitForAnyPress();
				System.exit(0);
			}
			
			//System.out.println("Round:");
			float rotate_angle1 =me.getPose().getHeading();						//get the current heading angle
			rotate_angle1 = Math.round(rotate_angle1/90.0)*90;					//get the round heading angle
			double forwardLength = (float) (((int)(rotate_angle1/90)%2)==1?0.00:0.03);	//decode the front distance
			me.correctFDistance(forwardLength);									//correct the front distance
			detect();															//detect the forward, left, right direction
			//me.updateInter();
			detectObstacle();													//update the forward, left, right obstacle
			if(!obstacleLeft) {													//if there is no obstacle in the left
				me.goNeighbour(3);												//then go left
			}else if(obstacleForward&&!obstacleRight) {							//if there is obstacle in the forward and no obstacle in the right
				me.goNeighbour(4);												//then go right
			}else if(obstacleForward&&obstacleRight){							//if there is obstacle in the both forward and righr direction
				me.goNeighbour(2);												//then go back
			}else {
				me.goNeighbour(1);												//else go straight
			}
			if(me.currentX==0 && me.currentY==0)									//if come back to the start point again
			{
				me.go_around_finished=true;										//then update the boolean flag
				break;
			}
			
		}
	}

	@Override
	public void suppress() {
		suppressed = true;

	}

	public void detectObstacle() {																//update the forward, left, right obstacle
		float rotate_angle = me.getPose().getHeading();											//get the heading position
		rotate_angle = Math.round(rotate_angle / 90.0) * 90;
		double rotate_radian = rotate_angle * Math.PI / 180;
		int cos_angle = (int) (Math.round(100*Math.cos(rotate_radian))/100);						//calculate the cos angle
		int sin_angle = (int) (Math.round(100*Math.sin(rotate_radian))/100);						//calculate the sin angle
		
		obstacleForward = ((me.currentX+cos_angle<0||me.currentX+cos_angle>=me.GRID_LNUMBER)||	//if out of the wall, then return true
						(me.currentY+sin_angle<0||me.currentY+sin_angle>=me.GRID_WNUMBER)) || 
						me.array_M[me.currentX+cos_angle][me.currentY+sin_angle]>0;				//else the array_M >0
		obstacleLeft = ((me.currentX+sin_angle<0||me.currentX+sin_angle>=me.GRID_LNUMBER)||		//if out of the wall, then return true
				(me.currentY-cos_angle<0||me.currentY-cos_angle>=me.GRID_WNUMBER)) || 
				me.array_M[me.currentX+sin_angle][me.currentY-cos_angle]>0;						//else the array_M >0
		obstacleRight = ((me.currentX-sin_angle<0||me.currentX-sin_angle>=me.GRID_LNUMBER)||		//if out of the wall, then return true
				(me.currentY+cos_angle<0||me.currentY+cos_angle>=me.GRID_WNUMBER)) || 
				me.array_M[me.currentX-sin_angle][me.currentY+cos_angle]>0;						//else the array_M >0
		//System.out.println(obstacleForward);
		//System.out.println(obstacleLeft);
		//System.out.println(obstacleRight);

	}
	public void detect() {																		//collect the distance info left, right, forward
		float distance_forward = me.getDistance();												//get the forward distance
		me.turnUMLeft();																			//then turn the sensor to left
		float distance_left = me.getDistance();													//get the left distance
		me.turnUMRight();																		//then turn the sensor to right
		float distance_right = me.getDistance();													//get get right distance
		me.turnUMForward();																		//then recover the sensor angle
		float rotate_angle = me.getPose().getHeading();
		rotate_angle = Math.round(rotate_angle / 90.0) * 90;
		double rotate_radian = rotate_angle * Math.PI / 180;										//calculate the current heading angle

		float Forward_x = distance_forward * (float) Math.cos(rotate_radian);						//calculate the distance in X axis
		float Forward_y = distance_forward * (float) Math.sin(rotate_radian);						//calculate the distance in Y axis

		float Left_x = distance_left * (float) Math.sin(rotate_radian);
		float Left_y = -distance_left * (float) Math.cos(rotate_radian);

		float Right_x = -distance_right * (float) Math.sin(rotate_radian);
		float Right_y = distance_right * (float) Math.cos(rotate_radian);

		me.updateMatrix(Forward_x, Forward_y);													//pass the data into the method of robot
		me.updateMatrix(Left_x, Left_y);
		me.updateMatrix(Right_x, Right_y);

		//abandoned part
		float forward_grid_length = (float) (((int) (rotate_angle / 90) % 2) == 1 ? me.GRID_WIDTH : me.GRID_LENGTH);
		float left_grid_length = (float) (((int) (rotate_angle / 90) % 2) == 1 ? me.GRID_LENGTH : me.GRID_WIDTH);

		obstacleForward = (int) (distance_forward / forward_grid_length) == 0;
		obstacleLeft = (int) (distance_left / left_grid_length) == 0;
		obstacleRight = (int) (distance_right / left_grid_length) == 0;
		
	}
	//abandoned method
	public boolean isReturn() {
		Pose currentPose = me.getPose();
		return Math.abs(currentPose.getX()) < 0.1 && Math.abs(currentPose.getY()) < 0.1
				&& currentPose.getHeading() != 0;
	}

	
}

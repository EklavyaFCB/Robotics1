package A1;

//PilotRobot.java
//
//Based on the SimpleRobot class, this provides access to the
//sensors, and constructs a MovePilot to control the robot.

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;

public class PilotRobot {

	private EV3TouchSensor leftBump, rightBump;
	private EV3IRSensor irSensor;
	private EV3UltrasonicSensor uSensor;
	private EV3ColorSensor cSensor;
	private SampleProvider leftSP, rightSP, distSP, colourSP;	
	private float[] leftSample, rightSample, distSample, colourSample; 
	private MovePilot pilot;
	final public double WALL_LENGTH = 1.93;				//the whole wall length
	final public double WALL_WIDTH = 1.53;				//the whole wall width
	final public int GRID_LNUMBER = 6;					//the grid length number
	final public int GRID_WNUMBER = 5;					//the grid width number
	final public double GRID_LENGTH = WALL_LENGTH/GRID_LNUMBER;	//single grid length
	final public double GRID_WIDTH = WALL_WIDTH/GRID_WNUMBER;		//single grid width
	private OdometryPoseProvider opp;
	public int[][] array_C,array_inter,array_M,array_final,array_been;
	public double[][] array_prob;
	/*
	public int[][] array_M = {{0,0,0,0,0},
							{0,0,0,0,0},
							{0,1,0,1,0},
							{0,0,0,0,0},
							{0,0,1,0,0},
							{0,0,0,0,0}};
	public int[][] array_final={{0,0,0,0,0},
								{0,0,0,0,0},
								{0,1,0,1,0},
								{0,0,0,0,0},
								{0,0,1,0,0},
								{0,0,0,0,0}};
	public int[][] array_been = {{1,1,1,1,1},
								{1,0,0,0,1},
								{1,0,0,0,1},
								{1,0,0,0,1},
								{1,0,0,0,1},
								{1,1,1,1,1}};
								*/
	public int currentX,currentY;								//current X position in grid, current Y position in grid
	private float correctAngle_distance = 10;					//the distance when correct left distance
	final public double SENSOR_WIDTH =  0.04;
	final public double SENSOR_LENGTH = 0.08;
	final public double CORRECT_DISTANCE_ANGLE = 38;
	public boolean go_around_finished = false;					//is go around behavior end
	public int end_X=-1;											//the end point X position
	public int end_Y=-1;											//the end point Y position
	
	
	public PilotRobot() {
		Brick myEV3 = BrickFinder.getDefault();

		leftBump = new EV3TouchSensor(myEV3.getPort("S2"));
		rightBump = new EV3TouchSensor(myEV3.getPort("S1"));
		uSensor = new EV3UltrasonicSensor(myEV3.getPort("S3"));
		cSensor = new EV3ColorSensor(myEV3.getPort("S4"));

		leftSP = leftBump.getTouchMode();
		rightSP = rightBump.getTouchMode();
		distSP = uSensor.getDistanceMode(); 		// effective range of the sensor in Distance mode is about 5 to 50 centimeters
		colourSP = cSensor.getRGBMode();
		
		leftSample = new float[leftSP.sampleSize()];		// Size is 1
		rightSample = new float[rightSP.sampleSize()];		// Size is 1
		distSample = new float[distSP.sampleSize()];		// Size is 1
		colourSample = new float[colourSP.sampleSize()];	// Size is 3
		
		// Set up the wheels by specifying the diameter of the
		// left (and right) wheels in centimeters, i.e. 3.25 cm
		// the offset number is the distance between the center
		// of wheel to the center of robot, i.e. half of track width
		// NOTE: this may require some trial and error to get right!!!
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 3.35).offset(-10.0);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 3.35).offset(10.0);
		Chassis myChassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		//Motor.A.setSpeed((float)33);
	    pilot = new MovePilot(myChassis);
	    
	    currentX=0;
	    currentY=0;
	    
	    opp = new OdometryPoseProvider(pilot);
	    opp.setPose(new Pose((float)GRID_LENGTH/2,(float)GRID_WIDTH/2,0)); 	//initial the start point as the central point
	    
	    array_M=new int[GRID_LNUMBER][GRID_WNUMBER];			
	    array_C=new int[GRID_LNUMBER][GRID_WNUMBER];
	    array_been=new int[GRID_LNUMBER][GRID_WNUMBER];		//the array for recording whether robot has been there
	    array_final=new int[GRID_LNUMBER][GRID_WNUMBER];
	    array_inter=new int[GRID_LNUMBER][GRID_WNUMBER];
	    array_prob= new double[GRID_LNUMBER][GRID_WNUMBER];
	    
	    array_been[currentX][currentY] = 1;								//the start point has been.
	    
	    
	}

	public void closeRobot() {											//close the robot and sensor
		leftBump.close();
		rightBump.close();
		irSensor.close();
		cSensor.close();
	}

	public boolean isLeftBumpPressed() {
 	leftSP.fetchSample(leftSample, 0);
 	return (leftSample[0] == 1.0);
	}
	
	public boolean isRightBumpPressed() {
 	rightSP.fetchSample(rightSample, 0);
 	return (rightSample[0] == 1.0);
	}
	
	public float getDistance() {
 	distSP.fetchSample(distSample, 0);
 	return distSample[0];
	}

	public float[] getColour() {
 	colourSP.fetchSample(colourSample, 0);
 	return colourSample;	// return array of 3 colours
	}
	
	public MovePilot getPilot() {
		return pilot;
	}
	
	
	public Pose getPose() {												//get the robot Pose								
		return opp.getPose();
	}
	
	public void setPose(Pose p) {										//set the robot pose
		opp.setPose(p);
	}
	
	public void rotateUpMotor(int angle){								//rotate the ultrasonic sensor
		Motor.A.rotate(angle, true);
	}
	
	public void turnUMLeft(){											//rotate the ultransonic sensor to the left
		Motor.A.rotateTo(90,false);
	}
	
	public void turnUMRight(){											//rotate the ultransonic sensor to the right
		Motor.A.rotateTo(-90,false);
	}
	
	public void turnUMForward() {										//rotate the ultransonic sensor to the forward
		Motor.A.rotateTo(0,false);
	}
	
	public float getUpMotor(){											
		return Motor.A.getPosition();
	}
	
	public boolean isUpMotorMoving() {									
		return Motor.A.isMoving();
	}

	public void goNeighbour(int direction) {								//the basic action, 1 for forward, 2 for back, 3 for left, 4 for right
		float rotate_angle1 = this.getPose().getHeading();				//get the current heading
		rotate_angle1 = Math.round(rotate_angle1/90.0)*90;				//get a round angle for easy calculation
		double forwardLength = (float) (((int)(rotate_angle1/90)%2)==1?0.00:0.03);	//use the heading angle to decide the distance between head and obstacle in the front
		switch(direction){
		case 1: 	this.correctFDistance(forwardLength);					//if go straight then adjust the distance
				this.correctDistance(0);									//correct the distance between left wall and robot
				break;
		case 2: 	this.correctFDistance(0.07);								//adjust the distance of forward
				this.correctDistance(1);									//correct the distance between left wall and robot
				pilot.rotate(180, false);								//if go back, then turn 180 degree
				break;
		case 3: 	this.correctFDistance(0.04);								//adjust the distance of forward
				this.correctDistance(1);									//correct the distance between left wall and robot 
				pilot.rotate(-90,false);									//if go left, then turn -90 degree
				break;
		case 4: 	this.correctFDistance(0.1);								//adjust the distance of forward
				this.correctDistance(1);									//correct the distance between left wall and robot
				pilot.rotate(90,false);									//if go right, then turn 90 degree
				break;
		}
		float rotate_angle = this.getPose().getHeading();					//get the current heading after rotating 
		rotate_angle = Math.round(rotate_angle/90.0)*90;					//get the round angle
		double rotate_radian = rotate_angle*Math.PI/180;					//get the radian of the angle
		double forwardLength1 = (float) (((int)(rotate_angle1/90)%2)==1?0.00:0.03);	//use the heading angle to decide the distance between head and obstacle in the front
		Pose currentPose = opp.getPose();
		opp.setPose(new Pose(currentPose.getX(),currentPose.getY(),rotate_angle));//reset the heading value to discard the error
		correctAngle(this.correctAngle_distance);						//correct the angle 
		
		float forward_grid_length = (float) (((int)(rotate_angle/90)%2)==1?GRID_WIDTH:GRID_LENGTH);	//calculate the grid length for current heading state
		pilot.travel(forward_grid_length*100-correctAngle_distance, false);							//then finish the left grid length, since correctAngle would also go a little distance
		
		currentX += Math.cos(rotate_radian);														//update the position and calculate the cos value of the current heading
		currentY += Math.sin(rotate_radian);														//update the position and calculate the sin value of the current heading
		array_been[currentX][currentY]=1;														//update the array_been as 1
		this.correctFDistance(forwardLength1);													//correct the distance

		if(getColour()[0]>0.015 && getColour()[0]<0.035) {										//find color paper
			end_X=currentX;
			end_Y=currentY;
			//System.out.println(end_X+" "+end_Y);
		}
		//this.correctDistance();
	}
	/*
	 * @parameter x the distance in x-axis
	 * @parameter y the distance in y-axis
	 */
	public void updateMatrix(float x, float y) {													//update the matrix based on the parameter
		int cell_x = (int)(x/GRID_LENGTH);														//transform the actual value into grid number
		int cell_y = (int)(y/GRID_WIDTH);
		int absolute_x = (int) Math.abs(x/GRID_LENGTH);											//get the absolute grid number
		int absolute_y = (int) Math.abs(y/GRID_WIDTH);
		int x_increment = x>=0?1:-1;																//get the direction
		int y_increment = y>=0?1:-1;
		if(Math.round(100*x)!=0) {																//if the x value is not equal to 0
			for(int i=1;i<=absolute_x&&x_increment*i+currentX<GRID_LNUMBER&&x_increment*i+currentX>=0;i++) {	//iterate the available grid in X-axis
				array_C[currentX+i*x_increment][currentY]++;										//count add 1
				array_M[currentX+i*x_increment][currentY]--;										//M array reduce 1
			}
			if(cell_x+currentX+x_increment<GRID_LNUMBER&&cell_x+currentX+x_increment>=0)			//update the obstacle grid
			{
				array_M[currentX+cell_x+x_increment][currentY]++;									//M array add 1
				array_C[currentX+cell_x+x_increment][currentY]++;
			}
		}
		if(Math.round(100*y)!=0) {																//if the y value is not equal to 0
			for(int i=1;i<=absolute_y&&y_increment*i+currentY<GRID_WNUMBER&&y_increment*i+currentY>=0;i++) {	//iterate the available grid in Y-axis
				array_C[currentX][currentY+i*y_increment]++;										//count add 1
				array_M[currentX][currentY+i*y_increment]--;										//M array reduce 1
			}
			
			if(cell_y+currentY+y_increment<GRID_WNUMBER&&cell_y+currentY+y_increment>=0)			//update the obstacle grid
			{
				array_M[currentX][currentY+cell_y+y_increment]++;									//M array add1
				array_C[currentX][currentY+cell_y+y_increment]++;
			}
		}
	}
	
	public void correctFDistance(double length) {												//correct the distance of front to control the car into the center of grid
		this.turnUMForward();																	//recover sensor heading
		float rotate_angle = this.getPose().getHeading();
		rotate_angle = Math.round(rotate_angle/90.0)*90;											//get the round angle
		double heading_grid_length =  (((int) (rotate_angle / 90) % 2) == 1 ? GRID_WIDTH : GRID_LENGTH);	//find the heading grid length
		double distance_should_be =  (heading_grid_length/2-length);								//the supposed position should be the half of grid then reduce the intended length
		
		double forward_distance =this.getDistance()%heading_grid_length;							//get the distance of front obstacle 
		//System.out.println("F_distance2:"+this.getDistance());
		//System.out.println(distance_should_be);
		if(Math.abs(forward_distance-distance_should_be)>GRID_WIDTH/2)							//if the difference is too big, then give up the correction
			return;
		this.pilot.travel((forward_distance-distance_should_be)*100);								//travel the distance
		
	}
	
	public void correctDistance(int type) {														//correct the distance between the left side, type 1 for turn around action, type 0 for forward action
		float rotate_angle = this.getPose().getHeading();
		rotate_angle = Math.round(rotate_angle/90.0)*90;
		double heading_grid_length =  (((int) (rotate_angle / 90) % 2) == 1 ? GRID_LENGTH : GRID_WIDTH);
		double distance_should_be =  ((heading_grid_length-SENSOR_WIDTH)/2);						//the supposed position should be the half grid
		this.turnUMLeft();																		//get the left side distance
		try {
			Thread.sleep(400);																	//sleep to make the sampling more accurate
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		double left_distance = this.getDistance()%heading_grid_length;							
		if(this.getDistance()>heading_grid_length)
		{
			this.turnUMForward();
			return;
		}

		double rotate_radian = CORRECT_DISTANCE_ANGLE * Math.PI / 180;
		if(type==0&&Math.abs(left_distance-distance_should_be)>0.008) {							//if the robot is going to go forward, then the tolerance could be larger
			pilot.rotate(this.CORRECT_DISTANCE_ANGLE);
			pilot.travel(100*(distance_should_be-left_distance)/Math.sin(rotate_radian),false);
			pilot.rotate(-this.CORRECT_DISTANCE_ANGLE);
			pilot.travel(-100*(distance_should_be-left_distance)/Math.sin(rotate_radian),false);
		}
		if(type==1&&Math.abs(left_distance-distance_should_be)>0.005) {							//if the robot is going to turn, then the tolerance should be smaller
			pilot.rotate(this.CORRECT_DISTANCE_ANGLE);
			pilot.travel(100*(distance_should_be-left_distance)/Math.sin(rotate_radian),false);
			pilot.rotate(-this.CORRECT_DISTANCE_ANGLE);
			pilot.travel(-100*(distance_should_be-left_distance)/Math.sin(rotate_radian),false);
		}
		this.turnUMForward();
		
	}
	
	public void correctAngle(double travelDistance){													//correct the move angle to straight
	    while(pilot.isMoving()) {
	        Thread.yield();  // wait till turn is complete or suppressed is called
	    }
	    turnUMLeft();																				//turn the ultrasonic sensor to left
		float sample_angle1 = getDistance();															//get the first sample
		pilot.travel(travelDistance);																//travel a little distance
	    while(pilot.isMoving()) {
	        Thread.yield();  // wait till turn is complete or suppressed is called
	    }
	    pilot.stop();																				//then stop
	    try {
			Thread.sleep(500);																		//sleep to make the sampling more accurate
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		float sample_angle2 = getDistance();															//get the second sample
		if(sample_angle1>GRID_LENGTH)																//if the sample is too long, then give up the action
			return;
		//System.out.println(Math.abs(sample_angle1-sample_angle2)+" "+(this.GRID_WIDTH-0.05)+" "+(Math.abs(sample_angle1-sample_angle2)>this.GRID_WIDTH-0.05));
		if(Math.abs(sample_angle1-sample_angle2)>0.05)												//if the difference is too long, then give up the action
			return;
		while(Math.abs(sample_angle1-sample_angle2)>0.005) {
			pilot.travel(-travelDistance);															//then reverse back to correct the angle
			
			
			double x = (double)(sample_angle2-sample_angle1)/(double)(travelDistance/100);			
			//System.out.println((sample_angle2-sample_angle1)+" "+(travelDistance/100));
			//System.out.println(x+" "+Math.asin(x));
			pilot.rotate(-180/3.14*Math.atan((double)(sample_angle2-sample_angle1)/(double)(travelDistance/100)));	//calculate the angle by using sample data
			
		    while(pilot.isMoving()) {
		        Thread.yield();  // wait till turn is complete or suppressed is called
		    }
		    pilot.stop();																			//then stop to get the sample
		    try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			sample_angle1 = getDistance();															//then get the first sample
			pilot.travel(travelDistance);															//travel a little distance
		    while(pilot.isMoving()) {
		        Thread.yield();  // wait till turn is complete or suppressed is called
		    }
		    pilot.stop();																			//then stop
		    try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			sample_angle2 = getDistance();															//then get the second sample
		}
		turnUMForward();																				//recover the up sensor motor
	}
	
	public void updateFinal() {													//calculate the final array;
		for(int i=0;i<GRID_LNUMBER;i++) {
			for(int j=0;j<GRID_WNUMBER;j++) {
				array_final[i][j] = (double) (array_M[i][j]+array_C[i][j])/(double)(2*array_C[i][j]) >0.5?1:0;
			}
		}
	}
	
	public void updateProbability() {													//calculate the final array;
		for(int i=0;i<GRID_LNUMBER;i++) {
			for(int j=0;j<GRID_WNUMBER;j++) {
				array_prob[i][j] = (double) (array_M[i][j]+array_C[i][j])/(double)(2*array_C[i][j]);
			}
		}
	}
	
	public void updateInter() {													//an abandoned method
		for(int i=0;i<GRID_LNUMBER;i++) {
			for(int j=0;j<GRID_WNUMBER;j++) {
				array_inter[i][j]=0;												//initial
				int count=0;
				count = i-1>=0&&array_M[i-1][j]<=0?count:count+1;
				count = i+1<GRID_LNUMBER&&array_M[i+1][j]<=0?count:count+1;
				count = j-1>=0&&array_M[i][j-1]<=0?count:count+1;
				count = j+1<GRID_WNUMBER&&array_M[i][j+1]<=0?count:count+1;
				if(count>=3||array_M[i][j]>0)
					array_inter[i][j]=1;
			}
		}
		
		for(int i=1;i<GRID_LNUMBER-1;i++) {
			for(int j=1;j<GRID_WNUMBER-1;j++) {
				if(array_M[i][j]>0) {
					if(array_M[i-1][j-1]>0){
						array_inter[i][j-1]=1;
						array_inter[i-1][j]=1;
					}
					if(array_M[i+1][j-1]>0){
						array_inter[i][j-1]=1;
						array_inter[i+1][j]=1;
					}
					if(array_M[i+1][j+1]>0){
						array_inter[i][j+1]=1;
						array_inter[i+1][j]=1;
					}
					if(array_M[i-1][j+1]>0){
						array_inter[i][j+1]=1;
						array_inter[i-1][j]=1;
					}
				}
			}
		}
	}

}
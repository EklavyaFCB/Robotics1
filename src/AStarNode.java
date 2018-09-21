package A1;
import java.util.*;

public class AStarNode {
	
	public AStarNode parent;

    private int x;
    private int y;
    private boolean obstacle;

	public int g; // Distance from start
	public int h; // Heuristic distance from destination
	public int f; // Sum f = g + h

	// Constructor
	public AStarNode(int x, int y, boolean obstacle) {
		this.x = x;
		this.y = y;
		this.obstacle=obstacle;
	}

	// Set Coordinates
	public void setCoordinates(int x, int y) {
		this.x = x;
		this.y = y;
	}

	// Get X 
    public int getX() {
    	return x;
    }

    // Get Y
    public int getY() {
    	return y;
    }

    // Check to see if its obstacle
    public boolean isObstacle() {
    	return obstacle;
    }

	// Parent Node
	public void setParent(AStarNode parent) {
		this.parent = parent;
	}

	public AStarNode getParent() {
		return parent;
	}

	// G Values
    public void setG(int g) {
    	this.g = g;
    }

    public void setG(AStarNode node) {
    	node.getG();
    }

    public int getG() {
    	return g;
    }

    // F Values
    public void calcF() {
    	this.f = g + h;
    }

    public int getF() {
    	return f;
    }

    // H Values -- Manhattan Distance
    public void calcH(AStarNode destPt) { 
		int dx = Math.abs(this.getX() - destPt.getX());
		int dy = Math.abs(this.getY() - destPt.getY());
		this.h = dx+dy;
    }

    public int getH() {
    	return h;
    }

}


package A1;
import java.util.*;
import java.util.Arrays;

public class AStarSearch<T extends AStarNode> {
	
	private int rows; 
	private int columns;
	
	private int[][] arr;
	private T[][] nodes;
	
	// Constructor 
	public AStarSearch(int rows, int columns, int[][] arr) {
		this.rows = rows;
		this.columns = columns;
		this.arr = arr;
		nodes = (T[][]) new AStarNode[rows][columns];
		initialiseNodes(arr);
	}

	// Initialise nodes
    public void initialiseNodes(int[][] arr) {
    	for (int i = 0; i < rows; i++) {
	        for (int j = 0; j < columns; j++) {
	            nodes[i][j] = createNode(i,j, arr[i][j]);
	        }
	    }
    }

    // Create nodes
    public T createNode(int x, int y, int obstacle) {
    	T node;
    	boolean boolObst = false;
    	if (obstacle==1) {
    		boolObst = true;
    	}
    	node = (T) new AStarNode(x,y,boolObst);
    	return node;
    }

    // Return node at given coordinates
    public final T getNode(int x, int y) {
    	return nodes[x][y];
    }

    // Destination coordinates
    int tarX;
    int tarY;

    // Assign destination coordinates
    public void getDestination() {
    	for (int i = 0; i < rows; i++) {
	        for (int j = 0; j < columns; j++) {
	            if (arr[i][j] == 2) {
	            	tarX = i;
	            	tarY = j;
	            }
	        }
	    }
    }

	// Unevaluated nodes list
	PriorityQueue<T> openList;
	// Evaluated nodes list
    HashSet<T> closedList;

	// Compare nodes -- required for PriorityQueue
    public class NodeComparator implements Comparator<T> {
        public int compare(T a, T b) {
            return Integer.compare(a.getF(), b.getF());
        }
    } 

    // Main part
	public List<Integer> aSearch(int startX, int startY) {

		openList = new PriorityQueue<T>(11, new NodeComparator());
	    closedList = new HashSet<T>();

		T currentNode = nodes[startX][startY];
		T destNode = nodes[tarX][tarY];

		// Info
		currentNode.parent = null;
		currentNode.setG(0);
		openList.add(currentNode);

		// Iterate
	    while (!openList.isEmpty()) {
	  		
	  		// Take top of openList
	  		currentNode = openList.poll();

	  		// If reached goal
	  		if ((currentNode.getX() == tarX) && (currentNode.getY() == tarY)) {

		        // Lists
		        ArrayList<T> nodePath = new ArrayList<T>(); // Contains path nodes
		        ArrayList<Integer> indexPath = new ArrayList<Integer>(); // Contains path indexes
		        nodePath = calcPath(nodes[startX][startY], currentNode);

		        for (int i = 0; i < nodePath.size(); i++) {
		        	// Add the Ys first, the Xs, bc we will reverse the list later
		        	indexPath.add(nodePath.get(i).getY());
		        	indexPath.add(nodePath.get(i).getX());
		        }

		        // Reverse so we get path in order
		        Collections.reverse(indexPath);
		        return indexPath;
	  		}

	  		// Iterate
	  		openList.remove(currentNode);
	  		closedList.add(currentNode);
	  		
	  		// Get list of neighbours
	  		List<T> adjNodes = getAdjacent(currentNode);
		  	
		  	// For each neighbour of current 
	  		for (int i=0; i<adjNodes.size(); i++) { 
		  		
		  		T adjNode = adjNodes.get(i);

		  		// Ignore evaluated node
		  		if (closedList.contains(adjNode)) { 
		  			continue;
		  		}

		  		// Discover new node
		  		if (!openList.contains(adjNode)) { 
		  			// If node isn't in openList
		  			adjNode.setParent(currentNode);
		  			adjNode.setG(currentNode);
		  			adjNode.calcH(nodes[tarX][tarY]);
		  			adjNode.calcF();
		  			openList.add(adjNode);
		  		} else { // If node is in openList
			  		if (adjNode.g < currentNode.g) {
			  			adjNode.setParent(currentNode);
			  			adjNode.setG(currentNode);
			  			currentNode = adjNode;
			  		}
		  		}
		  	}

		  	if (openList.isEmpty()) { // No path exists
		  		return new LinkedList<Integer>(); // Return empty list
		  	}
		}

		return null; // Unreachable

	}

	// Get Adjacent nodes if possible, and not already in closedList
	private List<T> getAdjacent(T node) {
		
		int x = node.getX();
		int y = node.getY();

		List<T> adj = new LinkedList<T>();

		T tempNode;

		if (x > 0) {
			tempNode = this.getNode((x-1),y);
			if (!closedList.contains(tempNode) && (!tempNode.isObstacle())) {
				adj.add(tempNode);
			}
		}

		if (x < (rows-1)) {
			tempNode = this.getNode((x+1),y);
			if (!closedList.contains(tempNode) && (!tempNode.isObstacle())) {
				adj.add(tempNode);
			}
		}

		if (y > 0) {
			tempNode = this.getNode(x,(y-1));
			if (!closedList.contains(tempNode) && (!tempNode.isObstacle())) {
				adj.add(tempNode);
			}
		}

		if (y < (columns-1)) {
			tempNode = this.getNode(x,(y+1));
			if (!closedList.contains(tempNode) && (!tempNode.isObstacle())) {
				adj.add(tempNode);
			}
		}

		return adj;
	}

    // Calculate path taken up till now
	private ArrayList<T> calcPath(T start, T goal) {
		// Source to destination path
		ArrayList<T> path = new ArrayList<T>();
		
		T node = goal;

		while (node.parent != null) {
			path.add(node);
			node = (T) node.getParent();
		}

		return path;
	}

}
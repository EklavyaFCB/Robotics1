package A1;
import java.util.*;
import java.util.Arrays;

public class AStar {
	public static void main(String[] args) {

	// This is a sample array - in the final implementation
	// this will be given to us through the Mapping class
	int[][] array_M =  {{1,1,1,1,0,1},
						{1,0,0,1,0,1},
						{1,1,0,1,0,1},
						{1,0,0,0,0,1},
						{1,1,0,0,2,1},
						{1,1,1,1,1,1}};

	// Get matrix info to parse
	int rows = array_M.length;
	int columns = array_M[0].length;

	// Instantiate with matrix info
        AStarSearch grid = new AStarSearch(rows,columns,array_M);
        
        // Methods

        // Search through matrix for destination (==2)
        grid.getDestination(); 
        // Find path, with starting node's i and j positions
        List path = grid.aSearch(1,1); 

        // Print ArrayList path
        System.out.println(path.size());

        // Print path in a table for simpler viewing
        System.out.print("\n");
        System.out.println("START");
        System.out.println("i | j");
        System.out.println("--|--");

        for (int i = 0; i < path.size(); i=i+2) {
        	System.out.print(path.get(i));
        	System.out.print(" | ");
        	System.out.println(path.get(i+1));
        }

        System.out.println("-END-");

	}
}
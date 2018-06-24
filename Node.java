

//import java.util.Vector;
import java.util.*;
public class Node extends ArrayList {
	
	public int index;
	public int predecessor_index;
	public double x_config;
	public double y_config;

	public Node(){}

	public Node(Node N){
		this.x_config=N.x_config;
		this.y_config=N.y_config;
		this.index=N.index;
		this.predecessor_index=N.predecessor_index;


	}
	
	
}



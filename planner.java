



import javafx.scene.canvas.GraphicsContext;


public class planner {

	public static void main1(int maxiter, double maxstep_size, double startXconfig, double startYconfig, double goalXconfig, double goalYconfig, double x_Cood, double y_Cood, double length, GraphicsContext gc) {
		RRT_Planner planner= new RRT_Planner();
		//Node newNode=new Node();

		SquareObject object = new SquareObject(length, (x_Cood+50), (y_Cood+50), gc);



		if (planner.solveQuery(maxiter, maxstep_size, (startXconfig+50), (startYconfig+50), (goalXconfig+50), (goalYconfig+50), object, gc))
		{
			System.out.println("SUCCESS" );
		}
		
	}
	
	
}

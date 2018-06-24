
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.control.Alert;
import javafx.scene.layout.StackPane;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;







public class Main extends Application {






    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setTitle("Basic RRT implementation");

        Group root = new Group();
        Canvas canvas = new Canvas(800,500);
        Canvas color = new Canvas(800,500);
        GraphicsContext gc;
        gc = canvas.getGraphicsContext2D();
        StackPane holder = new StackPane();
        holder.setStyle("-fx-background-color: ALICEBLUE");
        gc.scale(2.0, 2.0);
        gc.setLineWidth(5);
        gc.setStroke(Color.DEEPSKYBLUE);
        gc.strokeRect(25,25,150,150);






        Label max_itr = new Label("Enter max iterations");
        TextField max_itr_text = new TextField();
        max_itr_text.setPromptText("Enter +ve integer.");
        max_itr.setLayoutX(400);
        max_itr_text.setLayoutX(600);
        max_itr.setLayoutY(40);
        max_itr_text.setLayoutY(40);

        Label startX = new Label ("Enter start node X configuration");
        TextField startX_text = new TextField();
        startX_text.setPromptText("Enter value (1,100).");
        startX.setLayoutY(80);
        startX.setLayoutX(400);
        startX_text.setLayoutX(600);
        startX_text.setLayoutY(80);

        Label startY = new Label ("Enter start node Y configuration");
        TextField startY_text = new TextField();
        startY_text.setPromptText("Enter value (1,100).");
        startY.setLayoutY(120);
        startY.setLayoutX(400);
        startY_text.setLayoutX(600);
        startY_text.setLayoutY(120);

        Label goalX = new Label ("Enter goal node X configuration");
        TextField goalX_text = new TextField();
        goalX_text.setPromptText("Enter value (1,100).");
        goalX.setLayoutY(160);
        goalX.setLayoutX(400);
        goalX_text.setLayoutX(600);
        goalX_text.setLayoutY(160);

        Label goalY = new Label ("Enter goal node Y configuration");
        TextField goalY_text = new TextField();
        goalY_text.setPromptText("Enter value (1,100).");
        goalY.setLayoutY(200);
        goalY.setLayoutX(400);
        goalY_text.setLayoutX(600);
        goalY_text.setLayoutY(200);

        Label SquareX = new Label ("Enter Square object X configuration");
        TextField SquareX_text = new TextField();
        SquareX_text.setPromptText("Enter value (1,100).");
        SquareX.setLayoutY(240);
        SquareX.setLayoutX(400);
        SquareX_text.setLayoutX(600);
        SquareX_text.setLayoutY(240);

        Label SquareY = new Label ("Enter Square object Y configuration");
        TextField SquareY_text = new TextField();
        SquareY_text.setPromptText("Enter value (1,100).");
        SquareY.setLayoutY(280);
        SquareY.setLayoutX(400);
        SquareY_text.setLayoutX(600);
        SquareY_text.setLayoutY(280);

        Label SquareSize = new Label ("Enter Square object side length");
        TextField SquareSize_text = new TextField();
        SquareSize_text.setPromptText("Enter value (0,100).");
        SquareSize.setLayoutY(320);
        SquareSize.setLayoutX(400);
        SquareSize_text.setLayoutX(600);
        SquareSize_text.setLayoutY(320);

        Label max_step = new Label("Enter max step size");
        TextField max_step_text = new TextField();
        max_step_text.setPromptText("Enter +ve integer.");
        max_step.setLayoutX(400);
        max_step_text.setLayoutX(600);
        max_step.setLayoutY(360);
        max_step_text.setLayoutY(360);



        Button Run = new Button("RUN RRT");
        Run.setLayoutX(390);
        Run.setLayoutY(400);
        holder.getChildren().add(color);
        root.getChildren().add(holder);


        root.getChildren().add(Run);
        root.getChildren().add(max_itr);
        root.getChildren().add(max_itr_text);
        root.getChildren().add(startX);
        root.getChildren().add(startX_text);
        root.getChildren().add(startY);
        root.getChildren().add(startY_text);
        root.getChildren().add(goalX);
        root.getChildren().add(goalX_text);
        root.getChildren().add(goalY);
        root.getChildren().add(goalY_text);

        root.getChildren().add(SquareX);
        root.getChildren().add(SquareX_text);
        root.getChildren().add(SquareY);
        root.getChildren().add(SquareY_text);

        root.getChildren().add(SquareSize);
        root.getChildren().add(SquareSize_text);

        root.getChildren().add(max_step);
        root.getChildren().add(max_step_text);


        root.getChildren().add(canvas);
        Run.setOnAction(new EventHandler<ActionEvent>() {

            int clicked = 0;
            int maxItr=0;
            double maxStep, Start_X, Start_Y, Goal_X, Goal_Y, Square_X, Square_Y, Square_len;
            @Override
            public void handle(ActionEvent event) {

                 boolean count =true;
                if (clicked>0){

                    System.exit(0);
                }
                if(clicked==0){
                    clicked++;
                    if ((max_itr_text.getText() != null && !max_itr_text.getText().isEmpty())) {
                        /*Integer itr*/maxItr = Integer.valueOf(max_itr_text.getText());
                        /*System.out.println(itr);
                        maxItr=itr;*/
                    } else{
                        max_itr.setText("You have not left a comment.");
                        count=false;
                    }
                  if ((startX_text.getText() != null && !startX_text.getText().isEmpty())) {
                       /*Double strX*/Start_X = Double.valueOf(startX_text.getText());
                       /*System.out.println(strX);
                       Start_X=strX;*/
                   } else{
                        startX.setText("You have not left a comment.");
                        count=false;
                   }
                    if ((startY_text.getText() != null && !startY_text.getText().isEmpty())) {
                        /*Double strY*/Start_Y = Double.valueOf(startY_text.getText());
                        /*System.out.println(strY);
                        Start_Y=strY;*/
                    } else{
                        startY.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((goalX_text.getText() != null && !goalX_text.getText().isEmpty())) {
                        /*Double golX*/ Goal_X= Double.valueOf(goalX_text.getText());
                        /*System.out.println(golX);
                        Goal_X=golX;*/
                    } else{
                        goalX.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((goalY_text.getText() != null && !goalY_text.getText().isEmpty())) {
                        /*Double golY*/Goal_Y= Double.valueOf(goalY_text.getText());
                        /*System.out.println(golY);
                        Goal_Y=golY;*/
                    } else{
                        goalY.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((SquareX_text.getText() != null && !SquareX_text.getText().isEmpty())) {
                        /*Double sqrX*/Square_X = Double.valueOf(SquareX_text.getText());
                        /*System.out.println(sqrX);
                        Square_X=sqrX;*/
                    } else{
                        SquareX.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((SquareY_text.getText() != null && !SquareY_text.getText().isEmpty())) {
                        /*Double sqrY*/Square_Y = Double.valueOf(SquareY_text.getText());
                        /*System.out.println(sqrY);
                        Square_Y=sqrY;*/
                    } else{
                        SquareY.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((SquareSize_text.getText() != null && !SquareSize_text.getText().isEmpty())) {
                        /*Double sqrSize*/Square_len = Double.valueOf(SquareSize_text.getText());
                        //System.out.println(sqrSize);
                       // Square_len=sqrSize;
                    } else{
                        SquareSize.setText("You have not left a comment.");
                        count=false;
                    }
                    if ((max_step_text.getText() != null && !max_step_text.getText().isEmpty())) {
                       // Double step = Double.valueOf(max_step_text.getText());
                        maxStep= Double.valueOf(max_step_text.getText());
                       // System.out.println(step);
                      //  maxStep=step;
                    } else{
                        max_step.setText("You have not left a comment.");
                        count=false;
                    }



               // System.out.println("button action works");
                    if (count){
                planner.main1(maxItr,maxStep,Start_X,Start_Y,Goal_X,Goal_Y,Square_X,Square_Y,Square_len, gc);

                    }
                    else {
                        Alert alert = new Alert(Alert.AlertType.INFORMATION);
                        alert.setTitle("Information Dialog");
                        alert.setHeaderText("Cannot Run RRT");
                        alert.setContentText("You missed out on some parameters. Please exit and try again.");
                        alert.showAndWait();
                    }
                    Run.setText("Exit");
                    ;}





            }
        });










        primaryStage.setScene(new Scene(root));
        primaryStage.show();




    }


    public static void main(String[] args) {
        launch(args);

    }


}


package OM17;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;



/**
 * Created by OSPREY MINERS on 11/18/2016.
 */
public class Main extends Application {
    @Override
    public void start(Stage primaryStage) {
        Scene connectScene = new Scene(new ConnectGroup(), 800, 600);

        primaryStage.setScene(connectScene);
        primaryStage.setTitle("OM17 - MISSION CONTROL");
        primaryStage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }
}

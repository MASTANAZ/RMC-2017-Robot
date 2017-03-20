package om.mc.frontend;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import om.mc.Global;
import om.mc.backend.Backend;


/**
 * Created by Harris Newsteder on 3/6/17.
 */
public class MissionControl extends Application {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private Backend backend;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        launch(MissionControl.class, args);
    }

    public void start(Stage stage) {
        backend = new Backend();
        backend.start();

        // make the backend instance accessible from the entire program
        Global.setBackendInstance(backend);

        try {
            Parent root = FXMLLoader.load(getClass().getClassLoader().getResource("res/overview.fxml"));
            Scene scene = new Scene(root, Global.SCREEN_WIDTH, Global.SCREEN_HEIGHT);
            scene.getStylesheets().add("/res/material-fx-v0_3.css");
            stage.setScene(scene);
        } catch (Exception e) {
            e.printStackTrace();
        }

        stage.setTitle("OM17_MC");
        stage.show();
        stage.setMinWidth(stage.getWidth());
        stage.setMinHeight(stage.getHeight());

        stage.setOnCloseRequest(event -> cleanup());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void cleanup() {
        backend.stop();
    }
}

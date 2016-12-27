package OM.Client.Frontend;

import OM.Client.Backend.Backend;
import OM.Client.Global;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

/**
 * Created by Harris on 12/25/16.
 */
public class Client extends Application {
    private Stage stage = null;
    private Backend backend = null;

    @Override
    public void start(Stage stage) {

        System.out.println("--------------------------------------------------------------------------------");
        System.out.println("OM17-CLIENT");
        System.out.println("--------------------------------------------------------------------------------");

        this.stage = stage;

        try {
            Parent root = FXMLLoader.load(getClass().getClassLoader().getResource("res/connection_screen.fxml"));
            Scene scene = new Scene(root, Global.TARGET_WIDTH, Global.TARGET_HEIGHT);
            stage.setScene(scene);
        } catch (Exception e) {
            e.printStackTrace();
        }

        stage.setTitle("OM17-CLIENT");
        stage.show();

        stage.setMinWidth(stage.getWidth());
        stage.setMinHeight(stage.getHeight());

        stage.setOnCloseRequest(event -> cleanup());

        stage.setFullScreenExitHint("");
        stage.setFullScreen(false);

        backend = new Backend();
        backend.start();

        // give global a reference to this current instance of this class for the rest of the program to use
        Global.setClientInstance(this);
    }

    public static void main(String args[]) {
        launch(Client.class, args);
    }

    private void cleanup() {
        backend.stop();
    }

    public Backend getBackend() {
        return backend;
    }

    public Stage getStage() {
        return stage;
    }
}

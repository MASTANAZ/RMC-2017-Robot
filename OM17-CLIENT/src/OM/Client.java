package OM;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;

import java.io.IOException;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class Client extends Application {
    public static final int TARGET_WIDTH = 1280;
    public static final int TARGET_HEIGHT = 720;

    public enum State {
        CONNECT,
        MISSION
    }

    private Stage primaryStage;
    private Scene primaryScene;

    public Client() {
        Global.setClientInstance(this);
    }

    @Override
    public void start(Stage stage) throws Exception {
        primaryStage = stage;

        setState(State.CONNECT);

        stage.setTitle("OM17-CLIENT");
        stage.show();

        stage.setMinWidth(stage.getWidth());
        stage.setMinHeight(stage.getHeight());

        stage.setOnCloseRequest(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent event) {
                Global.setRunning(false);
            }
        });
    }

    public void setState(State state) {
        String fxml = "";

        switch (state) {
            case CONNECT:
                fxml = "res/connect_pane.fxml";
                break;
            case MISSION:
                fxml = "res/mission_pane.fxml";
                break;
            default:
                break;
        }

        Parent root = null;

        try {
            root = FXMLLoader.load(getClass().getClassLoader().getResource(fxml));
        } catch (IOException e) {
            e.printStackTrace();
        }

        Scene scene = null;

        if (getPrimaryScene() != null) {
            scene = new Scene(root, getPrimaryScene().getWidth(), getPrimaryScene().getHeight());
        } else {
            scene = new Scene(root, TARGET_WIDTH, TARGET_HEIGHT);
        }

        primaryStage.setScene(scene);

        primaryScene = scene;
    }

    public Scene getPrimaryScene() {
        return primaryScene;
    }

    public static void main(String[] args) {
        launch(Client.class, args);
    }
}

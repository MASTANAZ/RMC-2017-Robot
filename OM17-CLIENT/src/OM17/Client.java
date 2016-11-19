package OM17;

import javafx.application.Application;
import javafx.concurrent.Task;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.util.concurrent.ExecutorService;

/**
 * Created by OSPREY MINERS on 11/18/2016.
 */
public class Client extends Application {
    public static final int STATE_CONNECT = 0;
    public static final int STATE_CONTROL = 1;

    public static final int WINDOW_WIDTH = 1280;
    public static final int WINDOW_HEIGHT = 720;

    public int programState;

    private Scene primaryScene;
    private Stage primaryStage;

    @Override
    public void start(Stage primaryStage) {
        this.primaryStage = primaryStage;

        primaryStage.setMinHeight(WINDOW_HEIGHT);
        primaryStage.setMinWidth(WINDOW_WIDTH);
        primaryStage.setFullScreenExitHint("");
        primaryStage.setFullScreen(true);
        primaryStage.setTitle("OM17-CLIENT");
        primaryStage.show();

        changeState(STATE_CONNECT);
    }

    public void changeState(int newState) {
        programState = newState;

        Parent newParent = null;

        // choose appropriate parent for the new scene
        if (programState == STATE_CONNECT) {
            newParent = new ConnectPane(this);
        } else if (programState == STATE_CONTROL) {
            newParent = new ControlPane(this);
        // tried to switch to a state that doesn't exist
        } else {
            System.err.println("Attempted to switch to undefined program state.");
            System.exit(1);
        }

        // set the parent of the primaryScene to the new state
        if (primaryScene == null) {
            primaryScene = new Scene(newParent, WINDOW_WIDTH, WINDOW_HEIGHT);
        } else {
            primaryScene.setRoot(newParent);
        }

        // attach styles.css to current scene
        try {
            primaryScene.getStylesheets().add("res/styles.css");
        } catch (Exception e) {
            e.printStackTrace();
        }

        primaryStage.setScene(primaryScene);
    }

    public Stage getPrimaryStage() {
        return primaryStage;
    }

    public static void main(String[] args) {
        launch(args);
    }
}

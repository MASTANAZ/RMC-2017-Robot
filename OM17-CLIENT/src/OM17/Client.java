package OM17;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;

/**
 * Created by OSPREY MINERS on 11/18/2016.
 */
public class Client extends Application {
    public static final int TARGET_WIDTH = 1280;
    public static final int TARGET_HEIGHT = 720;

    public static final int STATE_CONNECT = 0;
    public static final int STATE_CONTROL = 1;

    public int programState;

    private Scene primaryScene;
    private Stage primaryStage;

    @Override
    public void start(Stage primaryStage) {
        this.primaryStage = primaryStage;

        changeState(STATE_CONNECT);

        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent event) {
                cleanup();
            }
        });
        primaryStage.setFullScreenExitHint("");
        primaryStage.setTitle("OM17-CLIENT");
        primaryStage.show();

        // ensure our window can't make our root content smaller than TARGET_WIDTH x TARGET_HEIGHT
        primaryStage.setMinWidth(primaryStage.getWidth());
        primaryStage.setMinHeight(primaryStage.getHeight());
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
            primaryScene = new Scene(newParent, TARGET_WIDTH, TARGET_HEIGHT);
        } else {
            primaryScene.setRoot(newParent);
        }

        // scale our root container so the GUI looks uniform at any resolution
        newParent.scaleXProperty().bind(primaryScene.widthProperty().divide(TARGET_WIDTH));
        newParent.scaleYProperty().bind(primaryScene.heightProperty().divide(TARGET_HEIGHT));

        // attach styles.css to current scene
        try {
            primaryScene.getStylesheets().add("res/styles.css");
        } catch (Exception e) {
            e.printStackTrace();
        }

        primaryStage.setScene(primaryScene);
    }

    public void cleanup() {
        RNI.cleanup();
    }

    public Stage getPrimaryStage() {
        return primaryStage;
    }

    public static void main(String[] args) {
        launch(args);
    }
}

package om.mc.frontend;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.canvas.Canvas;
import javafx.scene.layout.FlowPane;
import om.mc.Global;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris Newsteder on 3/7/2017.
 */
public class OverviewController implements Initializable {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @FXML private Canvas canvas;
    @FXML private FlowPane controlsPane;
    @FXML private FlowPane statusPane;
    @FXML private FlowPane networkPane;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        Global.getBackendInstance().getMission().setCanvas(canvas);

        try {
            controlsPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/controls.fxml")));
            statusPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/status.fxml")));
            statusPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/status.fxml")));
            networkPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/network.fxml")));
            networkPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/connection.fxml")));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
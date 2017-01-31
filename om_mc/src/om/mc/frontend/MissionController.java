package om.mc.frontend;

import om.mc.Global;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.canvas.Canvas;
import javafx.scene.layout.FlowPane;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris on 12/25/16.
 */
public class MissionController implements Initializable {
    @FXML private Canvas canvas;
    @FXML private FlowPane statusFlowPane;
    @FXML private FlowPane networkFlowPane;
    @FXML private FlowPane roundControlFlowPane;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        Global.getClientInstance().getBackend().getMission().setFieldCanvas(canvas);

        try {
            roundControlFlowPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/round_control_pane.fxml")));
            networkFlowPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/network_pane.fxml")));
            statusFlowPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/status_pane.fxml")));
            statusFlowPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/status_pane.fxml")));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

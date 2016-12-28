package OM.Client.Frontend;

import OM.Client.Backend.Mission;
import OM.Client.Backend.RNI;
import OM.Client.Global;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.paint.Color;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris on 12/25/16.
 */
public class MissionController implements Initializable {
    @FXML private Canvas canvas;
    @FXML private Label missionTimeLabel;
    @FXML private Label sentLabel;
    @FXML private Label receivedLabel;
    @FXML private Label totalLabel;
    @FXML private Button missionStartButton;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        Global.getClientInstance().getBackend().getMission().setFieldCanvas(canvas);
        missionTimeLabel.textProperty().bind(Global.getBackendInstance().getMission().timeProperty());
        sentLabel.textProperty().bind(RNI.sentProperty());
        receivedLabel.textProperty().bind(RNI.receivedProperty());
        totalLabel.textProperty().bind(RNI.totalProperty());
    }

    @FXML
    private void handleMissionStartAction(ActionEvent event) {
        Mission mission = Global.getBackendInstance().getMission();

        if (!mission.isActive()) {
            mission.start();
            missionStartButton.setText("STOP MISSION");
        } else {
            mission.stop();
            missionStartButton.setText("START MISSION");
        }
    }

    @FXML
    private void handleManualControlAction(ActionEvent event) {
        Global.getBackendInstance().getManualControl().findControllers();
    }
}

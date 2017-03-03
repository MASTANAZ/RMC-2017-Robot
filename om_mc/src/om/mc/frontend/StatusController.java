package om.mc.frontend;

import javafx.scene.control.Slider;
import om.mc.backend.Robot;
import om.mc.Global;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/30/2016.
 */
public class StatusController implements Initializable {
    @FXML Label nameLabel;
    @FXML Label xLabel;
    @FXML Label yLabel;
    @FXML Label orientationLabel;
    @FXML Slider lcvSlider;
    @FXML Slider rcvSlider;
    @FXML Label cpuTempLabel;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        Robot monitored = Global.getBackendInstance().getMission().getNextRobot();

        nameLabel.setText(monitored.getName());

        xLabel.textProperty().bind(monitored.getXProperty());
        yLabel.textProperty().bind(monitored.getYProperty());
        orientationLabel.textProperty().bind(monitored.getOrientationProperty());

        lcvSlider.valueProperty().bind(monitored.getLCVProperty());
        rcvSlider.valueProperty().bind(monitored.getRCVProperty());

        cpuTempLabel.textProperty().bind(monitored.getCPUTempProperty());
    }
}

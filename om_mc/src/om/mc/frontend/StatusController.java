package om.mc.frontend;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import om.mc.Global;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris Newsteder on 3/8/2017.
 */
public class StatusController implements Initializable {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @FXML public Slider lcvSlider;
    @FXML public Slider rcvSlider;
    @FXML public Label nameLabel;
    @FXML public Label cpuTempLabel;
    @FXML public Label xLabel;
    @FXML public Label yLabel;
    @FXML public Label thetaLabel;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        // grab either phobos or deimos and bind to the virtual robot class so the JavaFX properties can be updated
        Global.getBackendInstance().getMission().getRobot(Global.statusRobotIndex++).bindStatusController(this);
    }
}

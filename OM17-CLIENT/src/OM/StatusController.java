package OM;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/8/2016.
 */
public class StatusController implements Initializable {
    @FXML
    private Label positionLabel;

    private Robot monitored;

    public StatusController() {

    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        monitored = Round.getRobotA();
        positionLabel.textProperty().bind(monitored.getPositionPropertyString());
    }
}

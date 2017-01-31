package om.mc.frontend;

import om.mc.Global;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Button;
import javafx.scene.control.Label;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/30/2016.
 */
public class RoundControlController implements Initializable {
    @FXML Label currentTimeLabel;
    @FXML Label remainingTimeLabel;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        currentTimeLabel.textProperty().bind(Global.getBackendInstance().getMission().timeProperty());
        remainingTimeLabel.textProperty().bind(Global.getBackendInstance().getMission().remainingTimeProperty());
    }

    @FXML
    private void handleStartAction(ActionEvent event) {
        Button b = (Button)event.getSource();

        if (b.getText().equals("START")) {
            b.setText("STOP");
            Global.getBackendInstance().getMission().start();
        } else {
            b.setText("START");
            Global.getBackendInstance().getMission().stop();
        }
    }

    @FXML
    private void handleManualControlAction(ActionEvent event) {
        Global.getBackendInstance().getManualControl().findControllers();
    }
}

package om.mc.frontend;

import om.mc.backend.Network;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/30/2016.
 */
public class NetworkController implements Initializable {
    @FXML private Label sentLabel;
    @FXML private Label receivedLabel;
    @FXML private Label totalLabel;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        sentLabel.textProperty().bind(Network.sentProperty());
        receivedLabel.textProperty().bind(Network.receivedProperty());
        totalLabel.textProperty().bind(Network.totalProperty());
    }
}

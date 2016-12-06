package OM;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class ConnectController {
    @FXML
    void connect(ActionEvent event) {
        Global.getClientInstance().setState(Client.State.MISSION);
    }
}

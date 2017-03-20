package om.mc.frontend;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.control.Slider;
import om.mc.Global;
import om.mc.backend.Network;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris Newsteder on 3/10/2017.
 */
public class ConnectionController implements Initializable {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @FXML public ProgressIndicator phobosProgress;
    @FXML public ProgressIndicator deimosProgress;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        Network.bindConnectionController(this);
    }
}

package OM;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Alert;
import javafx.scene.control.DialogPane;
import javafx.scene.control.Label;
import javafx.scene.image.Image;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/8/2016.
 */
public class RoundControlController implements Initializable{

    public RoundControlController() {

    }

    @FXML
    private void handleButtonAction(ActionEvent event) {
        /*
        Alert alert = new Alert(Alert.AlertType.CONFIRMATION);
        alert.setTitle("EMERGENCY STOP WARNING");
        alert.setContentText("YOU ARE ABOUT TO PREMATURELY STOP THE ROBOT!\nPRESS OK TO CONFIRM");
        alert.setHeaderText("");
        DialogPane dp = alert.getDialogPane();
        //alert.setGraphic(new Image(this.getClass().getResource("res/warning_graphic.png").toString()));
        alert.showAndWait();
        */
    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {

    }
}

package OM.Client.Frontend;

import OM.Client.Backend.RNI;
import OM.Client.Global;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.text.Text;
import javafx.stage.Stage;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris on 12/25/16.
 */
public class ConnectionController implements Initializable {
    @FXML
    ProgressIndicator robotAConnectionProgressIndicator;

    @FXML
    ProgressIndicator robotBConnectionProgressIndicator;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        RNI.initialize();
        robotAConnectionProgressIndicator.progressProperty().bind(RNI.connectionAProperty());
        robotBConnectionProgressIndicator.progressProperty().bind(RNI.connectionBProperty());
    }

    @FXML
    void handleButtonAction(ActionEvent event) {
        Stage stage = (Stage)((Node)event.getSource()).getScene().getWindow();

        try {
            Parent root = FXMLLoader.load(getClass().getClassLoader().getResource("res/mission_screen.fxml"));
            Scene scene = new Scene(root, Global.TARGET_WIDTH, Global.TARGET_HEIGHT);
            stage.setScene(scene);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

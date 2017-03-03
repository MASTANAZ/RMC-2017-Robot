package om.mc.frontend;

import om.mc.backend.Network;
import om.mc.Global;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.ProgressIndicator;
import javafx.stage.Stage;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris on 12/25/16.
 */
public class ConnectionController implements Initializable {
    @FXML ProgressIndicator phobosConnIndicator;
    @FXML ProgressIndicator deimosConnIndicator;

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        phobosConnIndicator.progressProperty().bind(Network.phobosConnProperty());
        deimosConnIndicator.progressProperty().bind(Network.deimosConnProperty());
    }

    @FXML
    void handleButtonAction(ActionEvent event) {
        Stage stage = (Stage)((Node)event.getSource()).getScene().getWindow();

        boolean fullscreen = stage.isFullScreen();

        try {
            Parent root = FXMLLoader.load(getClass().getClassLoader().getResource("res/mission_screen.fxml"));
            Scene scene = new Scene(root, Global.TARGET_WIDTH, Global.TARGET_HEIGHT);
            stage.setScene(scene);
        } catch (Exception e) {
            e.printStackTrace();
        }

        stage.setFullScreen(fullscreen);
    }
}

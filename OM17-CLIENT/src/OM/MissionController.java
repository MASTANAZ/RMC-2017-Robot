package OM;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.layout.Pane;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class MissionController implements Initializable {
    @FXML
    private Pane fieldPane;

    @FXML
    private Pane statusPane;

    @FXML
    private Pane roundControlPane;

    public MissionController() {

    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        fieldPane.getChildren().add(new FieldCanvas(fieldPane));

        try {
            statusPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/robot_status_pane.fxml")));
            statusPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/robot_status_pane.fxml")));
            roundControlPane.getChildren().add(FXMLLoader.load(getClass().getClassLoader().getResource("res/round_control_pane.fxml")));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

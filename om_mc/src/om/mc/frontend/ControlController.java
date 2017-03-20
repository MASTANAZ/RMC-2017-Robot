package om.mc.frontend;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import javafx.stage.Popup;
import javafx.stage.PopupBuilder;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import om.mc.Global;
import om.mc.backend.Pose;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by Harris Newsteder on 3/20/2017.
 */
public class ControlController implements Initializable {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @FXML private ChoiceBox zoneChoice;
    @FXML private ChoiceBox directionChoice;
    @FXML private Button controlButton;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        zoneChoice.getItems().addAll("ZONE A", "ZONE B");
        directionChoice.getItems().addAll("NORTH", "SOUTH", "EAST", "WEST");

        controlButton.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if (zoneChoice.getValue() == null || directionChoice.getValue() == null) {
                    try {
                        Stage popup = new Stage();
                        popup.setScene(new Scene(FXMLLoader.load(getClass().getClassLoader().getResource("res/control_warning.fxml"))));
                        popup.show();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    return;
                }

                zoneChoice.setDisable(true);
                directionChoice.setDisable(true);

                controlButton.setText("ROUND STOP");
            }
        });

        zoneChoice.getSelectionModel().selectedIndexProperty().addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                switch((String)(zoneChoice.getItems().get((Integer) newValue))) {
                    case "ZONE A":

                        break;
                    case "ZONE B":

                        break;
                }
            }
        });

        directionChoice.getSelectionModel().selectedIndexProperty().addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                Pose phobos, deimos;
                phobos = Global.getBackendInstance().getMission().getRobot(0).getPose();
                deimos = Global.getBackendInstance().getMission().getRobot(1).getPose();

                switch((String)(directionChoice.getItems().get((Integer) newValue))) {
                    case "NORTH":
                        phobos.theta = (float)Math.PI;
                        deimos.theta = (float)Math.PI;
                        Global.getBackendInstance().getMission().getRobot(0).setPose(phobos);
                        Global.getBackendInstance().getMission().getRobot(1).setPose(deimos);
                        break;
                    case "SOUTH":
                        phobos.theta = 0.0f;
                        deimos.theta = 0.0f;
                        Global.getBackendInstance().getMission().getRobot(0).setPose(phobos);
                        Global.getBackendInstance().getMission().getRobot(1).setPose(deimos);
                        break;
                    case "EAST":
                        phobos.theta = (float)Math.PI * 3.0f / 2.0f;
                        deimos.theta = (float)Math.PI * 3.0f / 2.0f;
                        Global.getBackendInstance().getMission().getRobot(0).setPose(phobos);
                        Global.getBackendInstance().getMission().getRobot(1).setPose(deimos);
                        break;
                    case "WEST":
                        phobos.theta = (float)Math.PI / 2.0f;
                        deimos.theta = (float)Math.PI / 2.0f;
                        Global.getBackendInstance().getMission().getRobot(0).setPose(phobos);
                        Global.getBackendInstance().getMission().getRobot(1).setPose(deimos);
                        break;
                }
            }
        });
    }
}

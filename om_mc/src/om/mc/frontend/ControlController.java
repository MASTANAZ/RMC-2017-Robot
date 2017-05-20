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
import javafx.scene.control.*;
import javafx.scene.layout.VBox;
import javafx.stage.Popup;
import javafx.stage.PopupBuilder;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import om.mc.Global;
import om.mc.backend.ManualControl;
import om.mc.backend.Mission;
import om.mc.backend.Network;
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
    @FXML private Button manualControlButton;
    @FXML private CheckBox autonomyCheckBox;

    private Pose center;

    private int zone = 0;
    private int orientation = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        zoneChoice.getItems().addAll("ZONE A", "ZONE B");
        directionChoice.getItems().addAll("NORTH", "SOUTH", "EAST", "WEST");

        center = new Pose();

        center.x = 0.0f;
        center.y = 0.0f;
        center.theta = 0.0f;

        autonomyCheckBox.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if (autonomyCheckBox.isSelected()) {
                    try {
                        Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_AUTONOMY_START);
                    } catch (Exception e) {
                        //e.printStackTrace();
                    }
                } else {
                    try {
                        Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_AUTONOMY_STOP);
                    } catch (Exception e) {
                        //e.printStackTrace();
                    }
                }
            }
        });

        controlButton.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                handleControlButton();
            }
        });

        manualControlButton.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                ManualControl.findControllers();

                if (autonomyCheckBox.isSelected()) {
                    autonomyCheckBox.setSelected(false);
                    try {
                        Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_AUTONOMY_STOP);
                    } catch (Exception e) {
                        //e.printStackTrace();
                    }
                }
            }
        });

        zoneChoice.getSelectionModel().selectedIndexProperty().addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                handleZoneChoice((String)(zoneChoice.getItems().get((Integer) newValue)));
            }
        });

        directionChoice.getSelectionModel().selectedIndexProperty().addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                handleDirectionChoice((String)(directionChoice.getItems().get((Integer) newValue)));
            }
        });
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void handleControlButton() {
        // make sure the starting position and direction have been set before we attempt to start the robots
        if (zoneChoice.getValue() == null || directionChoice.getValue() == null) {
            try {
                Stage popup = new Stage();
                popup.setScene(new Scene(FXMLLoader.load(getClass().getClassLoader().getResource("res/control_warning.fxml"))));
                popup.show();
            } catch (Exception e) {
                //e.printStackTrace();
            }
            return;
        }

        Mission mission = Global.getBackendInstance().getMission();

        // button is being used to stop the current round
        if (mission.isRoundActive()) {
            zoneChoice.setDisable(false);
            directionChoice.setDisable(false);

            controlButton.setText("ROUND START");

            try {
                Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_ROUND_STOP);
            } catch (Exception e) {
                //e.printStackTrace();
            }

            mission.stopRound();
        // button is being used to start a new round
        } else {
            if (autonomyCheckBox.isSelected()) {
                try {
                    Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_AUTONOMY_START);
                } catch (Exception e) {
                    //e.printStackTrace();
                }
            } else {
                try {
                    Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_AUTONOMY_STOP);
                } catch (Exception e) {
                    //e.printStackTrace();
                }
            }

            zoneChoice.setDisable(true);
            directionChoice.setDisable(true);

            controlButton.setText("ROUND STOP");

            String out = "";

            int packed = (zone << 4) | orientation;

            out += (char)Network.S_STARTING_PARAMS + (char)packed;

            try {
                Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_STARTING_PARAMS + (char)packed);
                Network.writeString(Network.getClient(0).outStream, "" + (char)Network.S_ROUND_START);
            } catch (Exception e) {
                //e.printStackTrace();
            }

            mission.startRound();
        }
    }

    private void handleZoneChoice (String val) {
        switch(val) {
            case "ZONE A":
                zone = 0;
                break;
            case "ZONE B":
                zone = 1;
                break;
        }
    }

    private void handleDirectionChoice (String val) {
        Pose phobos, deimos;
        phobos = Global.getBackendInstance().getMission().getRobot(0).getPose();
        deimos = Global.getBackendInstance().getMission().getRobot(1).getPose();

        switch(val) {
            case "NORTH":
                orientation = 0;
                break;
            case "SOUTH":
                orientation = 1;
                break;
            case "EAST":
                orientation = 2;
                break;
            case "WEST":
                orientation = 3;
                break;
        }
    }
}

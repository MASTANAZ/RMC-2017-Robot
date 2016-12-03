package OM17;

import javafx.geometry.Insets;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;

/**
 * Created by OSPREY MINERS on 12/3/2016.
 */
public class ControlPane extends GridPane {
    HBox hbox;
    Button roundStartButton;
    ToggleButton manualControlButton;
    public ControlPane() {
        setPadding(new Insets(10, 10, 10, 10));

        roundStartButton = new Button();
        roundStartButton.setText("START ROUND");

        manualControlButton = new ToggleButton();
        manualControlButton.setText("MANUAL CONTROL");
        //manualControlButton.setDisable(true);

        hbox = new HBox();
        hbox.getChildren().addAll(roundStartButton, manualControlButton);

        hbox.setSpacing(5.0);

        getChildren().add(hbox);
    }
}

package OM;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;

import java.awt.event.MouseEvent;
import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class MissionController implements Initializable {
    @FXML
    private Pane fieldPane;

    public MissionController() {
        System.out.println("MISSION CONTROLLER");
    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        fieldPane.getChildren().add(new FieldCanvas(fieldPane));
    }
}

package OM17;

import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;

/**
 * Created by OSPREY MINERS on 12/1/2016.
 */
public class StatusPane extends GridPane {
    private Slider leftSlider, rightSlider;
    private HBox hbox;
    private Robot monitored;

    public StatusPane() {
        setPadding(new Insets(10, 10, 10, 10));

        hbox = new HBox();
        hbox.setSpacing(15.0);

        leftSlider = new Slider();
        leftSlider.setOrientation(Orientation.VERTICAL);
        leftSlider.setMin(-100);
        leftSlider.setMax(100);

        rightSlider = new Slider();
        rightSlider.setOrientation(Orientation.VERTICAL);
        rightSlider.setMin(-100);
        rightSlider.setMax(100);

        hbox.getChildren().add(new VBox(new Label(" L"), leftSlider));
        hbox.getChildren().add(new VBox(new Label(" R"), rightSlider));

        this.getChildren().add(hbox);
    }
}

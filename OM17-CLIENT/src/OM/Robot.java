package OM;

import javafx.application.Platform;
import javafx.beans.property.*;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Created by OSPREY MINERS on 12/8/2016.
 */
public class Robot {
    private float lcv, rcv;
    private float x, y;
    private float orientation;

    private StringProperty positionPropertyString;

    public Robot() {
        orientation = 0.0f;
        x = 0.75f;
        y = 1.5f;
        lcv = rcv = 0.0f;

        positionPropertyString = new SimpleStringProperty("X: " + String.format("%.2f", x) + " | Y: " + String.format("%.2f", y));
    }

    public void draw(GraphicsContext gc) {
        Affine old = gc.getTransform();

        Platform.runLater(new Runnable() {
            @Override
            public void run() {
                positionPropertyString.set("X: " + String.format("%.2f", x) + " | Y: " + String.format("%.2f", y));
            }
        });

        x += 0.01;

        gc.setFill(Color.CORNFLOWERBLUE);
        gc.translate(x, y);
        gc.rotate(orientation);
        gc.fillRect(-0.375f, -0.375f, 0.75f, 0.75f);

        gc.setTransform(old);
    }

    public StringProperty getPositionPropertyString() {
        return positionPropertyString;
    }
}

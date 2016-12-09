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

        positionPropertyString = new SimpleStringProperty("X: " + String.format("%.2fm", x) + "    Y: " + String.format("%.2fm", y));
    }

    public void draw(GraphicsContext gc) {
        // grab our transformation matrix before we render so we can reset it when we're done
        Affine old = gc.getTransform();

        // update our string property for the robot_status_pane on the JAVAFX main thread
        Platform.runLater(new Runnable() {
            @Override
            public void run() {
                positionPropertyString.set("X: " + String.format("%.2fm", x) + "    Y: " + String.format("%.2fm", y));
            }
        });

        gc.setFill(Color.CORNFLOWERBLUE);
        gc.translate(x, y);
        gc.rotate(orientation);
        gc.fillRect(-0.375f, -0.375f, 0.75f, 0.75f);

        gc.setTransform(old);
    }

    public StringProperty getPositionPropertyString() {
        return positionPropertyString;
    }

    public void setPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }
}

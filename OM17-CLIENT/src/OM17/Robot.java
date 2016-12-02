package OM17;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Created by OSPREY MINERS on 11/20/2016.
 */
public class Robot {
    public float x, y;
    public float orientation;
    public float leftChannel, rightChannel;

    public Robot() {
        x = 200;
        y = 200;
        orientation = 0;
        leftChannel = 0.0f;
        rightChannel = 0.0f;
    }

    public void update() {
        orientation += 0;
    }

    public void draw(GraphicsContext gc) {
        Affine tmp = gc.getTransform();
        gc.setFill(Color.BLACK);
        gc.translate(x, y);
        gc.rotate(orientation);
        gc.fillRect(-40, -40, 80, 80);
        gc.setTransform(tmp);
    }
}

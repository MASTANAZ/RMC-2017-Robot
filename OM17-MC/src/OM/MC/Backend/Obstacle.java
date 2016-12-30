package OM.MC.Backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Created by Harris on 12/26/16.
 */
public class Obstacle {
    private float x, y;
    private float diameter;

    private final Color COLOR_ROCK = new Color(0.5f, 0.4f, 0.4f, 0.9f);
    private final Color COLOR_CRATER = new Color(0.3f, 0.3f, 0.3f, 0.9f);

    public Obstacle() {
        x = 0.0f;
        y = 0.0f;
        diameter = 0.0f;
    }

    public void draw(GraphicsContext gc) {
        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        // draw the obstacle
        gc.setFill(COLOR_ROCK);
        gc.translate(x, y);
        gc.fillOval(-diameter / 2.0f, -diameter / 2.0f, diameter, diameter);

        // reset our graphics transformation matrix
        gc.setTransform(stack);
    }

    public void setPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void setDiameter(float diameter) {
        this.diameter = diameter;
    }
}

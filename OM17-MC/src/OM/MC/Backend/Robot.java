package OM.MC.Backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

import java.util.Map;

/**
 * Created by Harris on 12/25/16.
 */
public class Robot {
    private float x, y;
    private float orientation;
    private int lcv, rcv;
    private int propertyIndex;

    public Robot() {
        propertyIndex = -1;
    }

    public void tick(float dt) {
        if (propertyIndex == -1) return;

        // TODO: Fix this shit

        Map m = RNI.getPropertyMap(propertyIndex);

        if (m == null) return;

        int data[] = (int[])m.get(RNI.S_P_X);
        if (data != null) {
            float newX = ((float) ((int) data[3]) / 100.0f) + ((float) ((int) data[2]));
            setPosition(newX, getY());
        }

        data = (int[])m.get(RNI.S_P_Y);
        if (data != null) {
            float newY = ((float) ((int) data[3]) / 100.0f) + ((float) ((int) data[2]));
            setPosition(getX(), newY);
        }

        data = (int[])m.get(RNI.S_P_ORIENTATION);
        if (data != null) {
            float orientation = ((float) ((int) data[3]) / 100.0f) + ((float) ((int) data[2]));
            setOrientation(orientation);
        }
    }

    public void draw(GraphicsContext gc) {
        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        // draw the virtual robot with our current properties
        gc.setFill(Color.CORNFLOWERBLUE);
        gc.translate(x, y);
        gc.rotate((orientation * 180.0f) / Math.PI);
        gc.fillOval(-0.375f, -0.375f, 0.75f, 0.75f);
        gc.setFill(Color.BLACK);
        gc.fillRect(-0.25f, -0.025f, 0.5f, 0.05f);
        gc.fillOval(0.2f, -0.05f, 0.1f, 0.1f);

        // reset our graphics transformation matrix
        gc.setTransform(stack);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getOrientation() {
        return orientation;
    }

    public void setPosition(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void setOrientation(float orientation) {
        this.orientation = orientation;
    }

    public void setPropertyIndex(int index) {
        propertyIndex = index;
    }
}

package om.mc.backend;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

import java.util.ArrayList;

/**
 * Created by Harris Newsteder on 3/6/17.
 */
public class Mission {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static final int TARGET_DRAW_WIDTH = 1280;
    private static final int TARGET_DRAW_HEIGHT = 720;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private ArrayList<Robot> robotList;
    private Field field;
    private Canvas canvas;
    private GraphicsContext gc;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Mission() {
        robotList = new ArrayList<Robot>();
        field = new Field();

        Robot phobos = new Robot();
        phobos.setName("PHOBOS");
        robotList.add(phobos);

        Robot deimos = new Robot();
        deimos.setName("DEIMOS");
        robotList.add(deimos);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {
        field.tick(dt);

        // update all robots
        for (Robot r : robotList) {
            r.tick(dt);
        }
    }

    public void syncTick() {
        // make sure we have a valid canvas to draw to before we attempt to do any drawing operations
        if (gc == null || canvas == null) return;

        // the no transform graphics state, we use this to reset the graphics context at the end of drawing operations
        Affine noTransform = gc.getTransform();

        // scale to make sure our drawing operations happen inside the confines of the canvas
        gc.scale(canvas.getWidth() / (double)TARGET_DRAW_WIDTH, canvas.getHeight() / (double)TARGET_DRAW_HEIGHT);

        //
        gc.clearRect(0, 0, TARGET_DRAW_HEIGHT, TARGET_DRAW_HEIGHT);

        // scale factor for real-world meters to on screen pixels
        gc.scale(170.732, 170.732);

        field.draw(gc);

        // draw all robots
        for (Robot r : robotList) {
            r.draw(gc);
        }

        // reset our graphics transformation matrix
        gc.setTransform(noTransform);
    }

    public void setCanvas(Canvas canvas) {
        this.canvas = canvas;
        gc = canvas.getGraphicsContext2D();
    }

    public Robot getRobot(int index) {
        return robotList.get(index);
    }
}

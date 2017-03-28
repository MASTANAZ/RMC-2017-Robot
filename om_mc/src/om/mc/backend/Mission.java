package om.mc.backend;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.transform.Affine;
import om.mc.frontend.RoundMonitorController;

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
    private boolean roundActive;
    private float roundTime;

    private RoundMonitorController rmc;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Mission() {
        robotList = new ArrayList<Robot>();
        field = new Field();

        roundActive = false;

        roundTime = 0.0f;

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

        if (roundActive) {
            roundTime += dt;
        }
    }

    public void syncTick() {
        if (rmc != null) {
            rmc.elapsedLabel.setText("E+" + getTimeString(roundTime));
            rmc.remainingLabel.setText("R-" + getTimeString(600.0f - roundTime));
        }

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

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot getRobot(int index) {
        return robotList.get(index);
    }

    public Field getField() {
        return field;
    }

    public void startRound() {
        roundActive = true;
        roundTime = 0.0f;
    }

    public void stopRound() {
        roundActive = false;
    }

    public boolean isRoundActive() {
        return roundActive;
    }

    public void bindRoundMonitorController(RoundMonitorController rmc) {
        this.rmc = rmc;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private String getTimeString(float seconds) {
        int m = (int)Math.floor(seconds / 60);
        int s = (int)Math.floor(seconds) - (m * 60);
        int d = (int)Math.floor(seconds * 100) - (s * 100) - (m * 60 * 100);

        String mstring = Integer.toString(m);
        String sstring = Integer.toString(s);
        String dstring = Integer.toString(d);

        if (m < 10) mstring = "0" + mstring;
        if (s < 10) sstring = "0" + sstring;
        if (d < 10) dstring = "0" + dstring;

        return (mstring + ":" + sstring + ":" + dstring);
    }
}

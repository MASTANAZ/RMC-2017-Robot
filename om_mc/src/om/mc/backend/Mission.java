package om.mc.backend;

import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.transform.Affine;

import java.util.ArrayList;

/**
 * Created by Harris on 12/25/16.
 */
public class Mission {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private final int TARGET_WIDTH = 1280;
    private final int TARGET_HEIGHT = 720;

    private final float FIELD_WIDTH = 7.38f;
    private final float FIELD_HEIGHT = 3.78f;

    private final Color COLOR_REGOLITH = Color.web("#FFCC80");
    private final Color COLOR_OBSTACLE_AREA = Color.web("#FFB74D");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private boolean active = false;
    private float activeTime = 0.0f;
    private GraphicsContext gc = null;
    private Canvas fieldCanvas = null;
    private int currentRobot = 0;

    private ArrayList<Obstacle> obstacles = null;
    private ArrayList<Robot> robots = null;

    private SimpleStringProperty currentTimeProperty = null;
    private SimpleStringProperty remainingTimeProperty = null;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Mission() {
        currentRobot = 0;

        currentTimeProperty = new SimpleStringProperty("C+00:00:00");
        remainingTimeProperty = new SimpleStringProperty("R-10:00:00");

        obstacles = new ArrayList<Obstacle>();
        robots = new ArrayList<Robot>();

        Robot phobos = new Robot();
        phobos.setIdentifier("P");
        phobos.setName("PHOBOS");
        phobos.setDataModelIndex(0);

        Robot deimos = new Robot();
        deimos.setIdentifier("D");
        deimos.setName("DEIMOS");
        deimos.setDataModelIndex(1);

        robots.add(phobos);
        robots.add(deimos);

        active = false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {
        if (active) {
            activeTime += dt;
        }

        for (Robot robot : robots) {
            robot.tick(dt);
        }
    }

    public void synchronizedTick() {
        currentTimeProperty.set("C+" + getTimeString(activeTime));
        remainingTimeProperty.set("R-" + getTimeString(600.0f - activeTime));
        draw();
    }

    public void draw() {
        if (fieldCanvas == null || gc == null) return;

        float scaleX = (float)fieldCanvas.getWidth() / (float)TARGET_WIDTH;
        float scaleY = (float)fieldCanvas.getHeight() / (float)TARGET_HEIGHT;

        Affine noTransform = gc.getTransform();

        gc.scale(scaleX, scaleY);

        // clear all graphics from the pane
        gc.clearRect(0, 0, TARGET_WIDTH, TARGET_HEIGHT);

        // for centering our drawn field inside the pane we reside in
        gc.translate(10, 37.32);

        // scale factor for real world meters to on screen pixels
        gc.scale(170.732, 170.732);

        // field base
        gc.setFill(COLOR_REGOLITH);
        gc.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);

        // obstacle area
        gc.setFill(COLOR_OBSTACLE_AREA);
        gc.fillRect(1.5, 0, 2.94, 3.78);

        // draw phobos and deimos
        for (Robot robot : robots) {
            robot.draw(gc);
        }

        // draw all known obstacles
        for (Obstacle obstacle : obstacles) {
            obstacle.draw(gc);
        }

        // reset our graphics transformation
        gc.setTransform(noTransform);
    }

    public void start() {
        activeTime = 0.0f;
        active = true;
    }

    public void stop() {
        active = false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean isActive() {
        return active;
    }

    public Robot getRobot(int id) {
        return robots.get(id);
    }

    public Robot getNextRobot() {
        Robot tmp = robots.get(currentRobot);
        currentRobot++;
        return tmp;
    }

    public void setFieldCanvas(Canvas fieldCanvas) {
        this.fieldCanvas = fieldCanvas;
        gc = fieldCanvas.getGraphicsContext2D();

        // make sure the canvas always completely fills the cell it's in
        fieldCanvas.widthProperty().bind(((Pane)(fieldCanvas.getParent())).widthProperty());
        fieldCanvas.heightProperty().bind(((Pane)(fieldCanvas.getParent())).heightProperty());

        gc.setFont(new Font("Roboto Medium", 18));
    }

    public StringProperty timeProperty() {
        return currentTimeProperty;
    }

    public StringProperty remainingTimeProperty() {
        return remainingTimeProperty;
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

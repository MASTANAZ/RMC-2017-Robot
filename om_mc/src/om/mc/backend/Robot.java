package om.mc.backend;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

import javax.xml.crypto.Data;
import java.io.DataOutputStream;
import java.util.Map;

/**
 * Created by Harris on 12/25/16.
 */
public class Robot {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private final double X_POINTS[] =  {-0.375f, -0.375f, 0.375f, 0.375f};
    private final double Y_POINTS[] =  {-0.375f, 0.375f, 0.225f, -0.225f};
    private final int N_POINTS = 4;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private float x = 0.75f, y = 1.5f;
    private float orientation = 0.0f;
    private String identifier = "";
    private String name = "";
    private int dataModelIndex = -1;
    private double lcv = 0.0, rcv = 0.0;
    private float cpuTemp = 0.0f;
    private float gpuTemp = 0.0f;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // JAVAFX VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private SimpleStringProperty xProperty;
    private SimpleStringProperty yProperty;
    private SimpleStringProperty orientationProperty;
    private SimpleDoubleProperty lcvProperty;
    private SimpleDoubleProperty rcvProperty;
    private SimpleStringProperty cpuTempProperty;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot() {
        xProperty = new SimpleStringProperty("X: 0.00m");
        yProperty = new SimpleStringProperty("Y: 0.00m");
        orientationProperty = new SimpleStringProperty("θ: 0°");
        lcvProperty = new SimpleDoubleProperty(0.0);
        rcvProperty = new SimpleDoubleProperty(0.0);
        cpuTempProperty = new SimpleStringProperty("CPU: 49.6°C");
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {
        if (dataModelIndex == -1) return;

        setX(DataModel.getData(dataModelIndex, DataModel.PROP_X));
        setY(DataModel.getData(dataModelIndex, DataModel.PROP_Y));
        setOrientation(DataModel.getData(dataModelIndex, DataModel.PROP_ORIENTATION));
        lcv = DataModel.getData(dataModelIndex, DataModel.PROP_LCV);
        rcv = DataModel.getData(dataModelIndex, DataModel.PROP_RCV);
        cpuTemp = (float)DataModel.getData(dataModelIndex, DataModel.CPU_TEMP);
    }

    public void draw(GraphicsContext gc) {
        xProperty.set("X: " + String.format("%.2f", x) + "m");
        yProperty.set("Y: " + String.format("%.2f", y) + "m");
        orientationProperty.set("θ: " + (int)((-(orientation - (Math.PI * 2.0f))) * 180.0f / Math.PI) + "°");
        lcvProperty.set(lcv);
        rcvProperty.set(rcv);
        cpuTempProperty.set("CPU: " + cpuTemp + "°C");

        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        gc.translate(x, y);
        gc.rotate((orientation * 180.0f) / Math.PI);

        // robot body
        gc.setFill(Color.CORNFLOWERBLUE);
        gc.fillPolygon(X_POINTS, Y_POINTS, N_POINTS);
        gc.scale(0.85f, 0.85f);
        gc.setFill(Color.LIGHTSKYBLUE);
        gc.fillPolygon(X_POINTS, Y_POINTS, N_POINTS);
        gc.scale(100.0f / 85.0f, 100.0f / 85.0f);

        gc.setLineWidth(0.02f);

        // forward vector
        gc.setStroke(Color.INDIANRED);
        gc.strokeLine(-0.375f, 0.0f, -0.6f, 0.0f);

        // backwards vector (camera face)
        gc.setStroke(Color.CORNFLOWERBLUE);
        gc.strokeLine(0.375f, 0.0f, 0.6f, 0.0f);

        // robot name
        gc.translate(-0.08f, 0.093f);
        gc.scale(1.0f / 65.0f, 1.0f / 65.0f);
        gc.setFill(Color.BLACK);
        gc.fillText(this.identifier, 0, 0);
        gc.scale(65.0f, 65.0f);

        // reset our graphics transformation matrix
        gc.setTransform(stack);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public String getName() {
        return name;
    }

    public StringProperty getXProperty() {
        return xProperty;
    }

    public StringProperty getYProperty() {
        return yProperty;
    }

    public StringProperty getOrientationProperty() {
        return orientationProperty;
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

    public void setName(String name) {
        this.name = name;
    }

    public void setDataModelIndex(int index) {
        this.dataModelIndex = index;
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

    public void setIdentifier(String name) {
        this.identifier = name;
    }

    public void setOrientation(float orientation) {
        this.orientation = orientation;
    }

    public DoubleProperty getLCVProperty() {
        return lcvProperty;
    }

    public DoubleProperty getRCVProperty() {
        return rcvProperty;
    }

    public StringProperty getCPUTempProperty() {
        return cpuTempProperty;
    }
}

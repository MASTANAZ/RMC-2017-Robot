package om.mc.backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.transform.Affine;
import om.mc.frontend.StatusController;
import sun.nio.ch.Net;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by Harris Newsteder on 3/6/17.
 */
public class Robot {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private final Color COLOR_CAMERA_FOV = Color.web("#6FC2F2", 0.25);
    private final Font FONT = new Font(0.25);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private Map propertyMap;

    private int lcv;
    private int rcv;
    private String name;
    private Pose pose;
    private float cpuTemp;

    private int lcvOld, rcvOld;

    private float networkTimer;

    private ArrayList<Integer> statementList;

    private StatusController controller;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot() {
        lcv     = 0;
        rcv     = 0;
        name    = "";
        pose    = new Pose();
        pose.x = 2.0f;
        pose.y = 1.0f;
        cpuTemp = 0.0f;

        networkTimer = 0.0f;

        lcvOld = lcv;
        rcvOld = rcv;


        statementList = new ArrayList<Integer>();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {
        parseStatements();

        networkTimer += dt;

        if (networkTimer > 0.1f && (Network.getClientSize() > 0)) {
            String out = "";

            if (lcv != lcvOld) {
                out += (char)Network.S_LCV;
                out += (char)((int)(lcv + 100));
                lcvOld = lcv;
            }

            if (rcv != rcvOld) {
                out += (char)Network.S_RCV;
                out += (char)((int)(rcv + 100));
                rcvOld = rcv;
            }

            try {
                Network.writeString(Network.getClient(0).outStream, out);
            } catch (Exception e) {
                e.printStackTrace();
            }
            networkTimer -= 0.1f;
        }
    }

    public void draw(GraphicsContext gc) {
        // update the status monitor
        if (controller != null) {
            controller.lcvSlider.setValue((double)lcv);
            controller.rcvSlider.setValue((double)rcv);
            controller.xLabel.setText("X: " + pose.x + "m");
            controller.yLabel.setText("Y: " + pose.y + "m");
            controller.thetaLabel.setText("θ: " + (int)(pose.theta * 180.0f / Math.PI) + "°");
            controller.cpuTempLabel.setText("CPU: " + cpuTemp + "°C");
        }

        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        Pose p = getPose();

        gc.translate(p.x, p.y);
        gc.rotate((p.theta * 180.0f) / Math.PI);

        // robot body
        gc.setFill(Color.CORNFLOWERBLUE);
        gc.fillRect(-0.375f, -0.375f, 0.75f, 0.75f);

        gc.setLineWidth(0.02f);

        // forward vector
        gc.setStroke(Color.INDIANRED);
        gc.strokeLine(0.375f, 0.0f, 0.6f, 0.0f);

        // camera FOV
        gc.setStroke(COLOR_CAMERA_FOV);
        gc.strokeLine(-0.375f, 0.0f, -15.375f, 6.995f);
        gc.strokeLine(-0.375f, 0.0f, -15.375f, -6.995f);

        gc.rotate((p.theta * -180.0f) / Math.PI);
        gc.translate(-0.05, 0.05);
        gc.setFill(Color.BLACK);
        gc.setFont(FONT);
        gc.fillText("P", 0, 0);

        // reset our graphics transformation matrix
        gc.setTransform(stack);
    }

    public void pushStatement(int statement) {
        statementList.add(statement);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void parseStatements() {
        int cbval, plen;

        while (statementList.size() > 1) {
            cbval = statementList.get(0);
            plen = statementList.size();

            switch (cbval) {
                case Network.S_CPU_TEMP:
                    if (plen < 3) break;

                    cpuTemp = (float)statementList.get(1) + ((float)statementList.get(2) / 100.0f);

                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);

                    System.out.println(cpuTemp);

                    break;
                case Network.S_POSE:
                    if (plen < 4) break;

                    // SUPA PACKED DATA
                    pose.x = (float)((statementList.get(1) & 0xF0) >> 4) + ((float)(statementList.get(1) & 0x0F) / 10.0f);
                    pose.y = (float)((statementList.get(2) & 0xF0) >> 4) + ((float)(statementList.get(2) & 0x0F) / 10.0f);
                    pose.theta = (float)((statementList.get(3) & 0xF0) >> 4) + ((float)(statementList.get(3) & 0x0F) / 10.0f);

                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);

                    break;
                case Network.S_LCV:
                case Network.S_RCV:
                    break;
                default:
                    break;
            }
        }
    }

    public void bindStatusController(StatusController sc) {
        controller = sc;
        controller.nameLabel.setText(name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Pose getPose() {
        return pose;
    }

    public void setLCV(int lcv) {
        this.lcv = lcv;
    }

    public void setRCV(int rcv) {
        this.rcv = rcv;
    }

    public int getLCV() {
        return lcv;
    }

    public int getRCV() {
        return rcv;
    }
}

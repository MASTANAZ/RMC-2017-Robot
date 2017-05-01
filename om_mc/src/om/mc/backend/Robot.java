package om.mc.backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.transform.Affine;
import om.mc.Global;
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

    public static final int ID_PHOBOS = 0;
    public static final int ID_DEIMOS = 1;

    private final float ROBOT_WIDTH = 0.889f;
    private final float ROBOT_WIDTH_HALF = ROBOT_WIDTH / 2.0f;
    private final float ROBOT_HEIGHT = 0.7239f;
    private final float ROBOT_HEIGHT_HALF = ROBOT_HEIGHT / 2.0f;

    private final int STATE_LNCH = 0;
    private final int STATE_TTES = 1;
    private final int STATE_EXCV = 2;
    private final int STATE_TTDS = 3;
    private final int STATE_DEPO = 4;

    private final int CONTROL_STATE_TRVL = 0;
    private final int CONTROL_STATE_EXCV = 1;
    private final int CONTROL_STATE_DEPO = 2;

    private final String STATE_DESCRIPTIONS[] = {
            "LAUNCH CONFIGURATION",
            "TRAVELLING TO EXCAVATION SITE",
            "EXCAVATING",
            "TRAVELLING TO DEPOSITION BIN",
            "DEPOSITING PAYLOAD"
    };
    private final String CONTROL_STATE_DESCRIPTIONS[] = {
            "CONTROL STATE TRAVEL",
            "CONTROL STATE EXCAVATE",
            "CONTROL STATE DEPOSIT"
    };

    private final Color COLOR_CAMERA_FOV = Color.web("#6FC2F2", 0.25);
    private final Font FONT = new Font(0.25);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private int lcv;
    private int rcv;
    private String name;
    private Pose pose;
    private float cpuTemp;

    private int id;
    private Client boundClient;

    private int state, controlState;

    private int lcvOld, rcvOld;
    private float networkTimer;

    private ArrayList<Integer> statementList;

    private StatusController controller;

    private int removeme = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(int id) {
        lcv     = 0;
        rcv     = 0;
        name    = "";
        pose    = new Pose();
        pose.x = -1.0f;
        pose.y = -1.0f;
        pose.theta = 0.0f;

        cpuTemp = 0.0f;

        this.id = id;
        this.boundClient = Network.getClient(id);

        state = STATE_LNCH;
        controlState = CONTROL_STATE_TRVL;

        networkTimer = 0.0f;

        // TODO: remove
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

        if (networkTimer > 0.1f && boundClient.alive) {
            String out = "";

            if (lcv != lcvOld) {
                out += (char)Network.S_MC1;
                out += (char)((int)(lcv + 100));
                lcvOld = lcv;
            }

            if (rcv != rcvOld) {
                out += (char)Network.S_MC2;
                out += (char)((int)(rcv + 100));
                rcvOld = rcv;
            }

            try {
                Network.writeString(boundClient.outStream, out);
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
            controller.thetaLabel.setText("θ: " + pose.theta + "°");
            controller.cpuTempLabel.setText("CPU: " + cpuTemp + "°C");
            controller.stateLabel.setText(STATE_DESCRIPTIONS[state]);
            controller.controlStateLabel.setText(CONTROL_STATE_DESCRIPTIONS[controlState]);
        }

        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        Pose p = getPose();

        gc.translate(0.0f, Field.FIELD_HEIGHT);
        gc.translate(p.x, -p.y);
        gc.rotate(-p.theta);

        // robot body
        gc.setFill(Color.CORNFLOWERBLUE);
        gc.fillRect(-ROBOT_WIDTH_HALF, -ROBOT_HEIGHT_HALF, ROBOT_WIDTH, ROBOT_HEIGHT);

        gc.setLineWidth(0.02f);

        // forward vector
        gc.setStroke(Color.INDIANRED);
        gc.strokeLine(ROBOT_WIDTH_HALF, 0.0f, ROBOT_WIDTH + 0.1f, 0.0f);

        // camera FOV
        gc.setStroke(COLOR_CAMERA_FOV);
        gc.strokeLine(-0.375f, 0.0f, -15.375f, 6.995f);
        gc.strokeLine(-0.375f, 0.0f, -15.375f, -6.995f);

        gc.rotate(p.theta);
        gc.translate(-0.05, 0.05);
        gc.setFill(Color.BLACK);
        gc.setFont(FONT);
        gc.fillText("" + name.charAt(0), 0, 0);

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

                    break;
                case Network.S_POSE:
                    if (plen < 4) break;

                    // SUPA PACKED DATA
                    pose.x = (float)((statementList.get(1) & 0xF0) >> 4) + ((float)(statementList.get(1) & 0x0F) / 10.0f);
                    pose.y = (float)((statementList.get(2) & 0xF0) >> 4) + ((float)(statementList.get(2) & 0x0F) / 10.0f);
                    pose.theta = (float)(statementList.get(3) - 90);

                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);

                    break;
                case Network.S_MC1:
                case Network.S_MC2:
                    break;
                case Network.S_STATE:
                    if (plen < 2) break;

                    state = statementList.get(1);

                    statementList.remove(0);
                    statementList.remove(0);

                    break;

                // TODO: UPDATE SO NETWORK CAN HANDLE ACTUAL COST VALUES INSTEAD OF JUST -1
                case Network.S_CELL_COST:
                    if (plen < 3) break;

                    Global.getBackendInstance().getMission().getField().updateCostMap(statementList.get(1), statementList.get(2),-1.0f);

                    statementList.remove(0);
                    statementList.remove(0);
                    statementList.remove(0);

                    break;
                case Network.S_CONTROL_STATE:
                    if (plen < 2) break;

                    controlState = statementList.get(1);

                    statementList.remove(0);
                    statementList.remove(0);

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

    public void toggleControlState() {
        this.controlState++;

        if (controlState > 2) controlState = 0;

        String out = "";

        out += (char)Network.S_CONTROL_STATE;
        out += (char)controlState;

        try {
            Network.writeString(boundClient.outStream, out);
        } catch (Exception e) {
            e.printStackTrace();
        }
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

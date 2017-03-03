package om.mc.backend;

import javafx.application.Platform;
import javafx.concurrent.Task;
import om.mc.Global;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by OSPREY MINERS on 1/2/2017.
 */
public class DataModel {
    private static ArrayList<Map> dataList;
    private static float sendTimer = 0.0f;

    private static double lastLCV = 0.0, lastRCV = 0.0;

    private static final int NETWORK_SEND_RATE = 10;
    private static final float NETWORK_SEND_TIME = 1.0f / (float)NETWORK_SEND_RATE;

    public static final int PROP_X           = Network.S_P_X;
    public static final int PROP_Y           = Network.S_P_Y;
    public static final int PROP_ORIENTATION = Network.S_P_ORIENTATION;
    public static final int PROP_LCV         = Network.S_P_LCV;
    public static final int PROP_RCV         = Network.S_P_RCV;
    public static final int OBSTACLE         = Network.S_OBSTACLE;
    public static final int CPU_TEMP         = Network.S_CPU_TEMP;

    public static void initialize() {
        dataList = new ArrayList<>();

        // phobos
        dataList.add(new HashMap());

        // deimos
        dataList.add(new HashMap());

        // set all property values initially to zero
        for (int i = 0; i < dataList.size(); ++i) {
            putData(i, PROP_X, (float)0.0f);
            putData(i, PROP_Y, (float)0.0f);
            putData(i, PROP_ORIENTATION, (float)0.0f);
            putData(i, PROP_LCV, (double)0.0f);
            putData(i, PROP_RCV, (double)0.0f);
            putData(i, CPU_TEMP, (float)0.0f);
        }
    }

    public static void tick(float dt) {
        sendTimer += dt;
        if (sendTimer > 0.1f) {
            double lcv = (double)dataList.get(0).get(PROP_LCV);
            double rcv = (double)dataList.get(0).get(PROP_RCV);

            if (lcv != lastLCV) {
                char[] data = new char[1];
                data[0] = (char)((int)(lcv + 100.0));
                Network.state(0, PROP_LCV, data);
                lastLCV = lcv;
            }
            if (rcv != lastRCV) {
                char[] data = new char[1];
                data[0] = (char)((int)(rcv + 100.0));
                Network.state(0, PROP_RCV, data);
                lastRCV = rcv;
            }
            sendTimer -= 0.1f;
        }
    }

    public static void putData(int index, int property, int packed[]) {
        Object unpacked = null;

        switch (property) {
            case PROP_Y:
                unpacked = ((float) ((int) packed[3]) / 100.0f) + ((float) ((int) packed[2]));
                unpacked = (float)unpacked + (float)0.42f;
                break;
            case PROP_X:
            case CPU_TEMP:
            case PROP_ORIENTATION:
                unpacked = ((float) ((int) packed[3]) / 100.0f) + ((float) ((int) packed[2]));
                break;
            default:
                break;
        }

        dataList.get(index).put(property, unpacked);
    }

    public static void putData(int index, int property, Object item) {
        dataList.get(index).put(property, item);
    }

    public static <T> T getData(int index, int property) {
        return (T)dataList.get(index).get(property);
    }
}

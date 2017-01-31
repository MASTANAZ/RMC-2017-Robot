package om.mc.backend;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by OSPREY MINERS on 1/2/2017.
 */
public class DataModel {
    private static ArrayList<Map> dataList;

    public static final int PROP_X           = RNI.S_P_X;
    public static final int PROP_Y           = RNI.S_P_Y;
    public static final int PROP_ORIENTATION = RNI.S_P_ORIENTATION;

    public static void initialize() {
        dataList = new ArrayList<>();

        // phobos
        dataList.add(new HashMap());

        // deimos
        dataList.add(new HashMap());

        // set all property values initially to zero
        for (Map m : dataList) {
            m.put(PROP_X, 0.0f);
            m.put(PROP_Y, 0.0f);
            m.put(PROP_ORIENTATION, 0.0f);
        }
    }

    public static void putData(int index, int property, int packed[]) {
        Object unpacked = null;

        switch (property) {
            case PROP_X:
            case PROP_Y:
            case PROP_ORIENTATION:
                unpacked = ((float) ((int) packed[3]) / 100.0f) + ((float) ((int) packed[2]));
        }

        dataList.get(index).put(property, unpacked);
    }

    public static <T> T getData(int index, int property) {
        return (T)dataList.get(index).get(property);
    }
}

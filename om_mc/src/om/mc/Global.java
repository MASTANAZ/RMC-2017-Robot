package om.mc;

import om.mc.backend.Backend;
import om.mc.frontend.Client;

/**
 * Created by Harris on 12/25/16.
 */
public class Global {
    public static final int TARGET_WIDTH = 1280;
    public static final int TARGET_HEIGHT = 720;
    public static final int PHOBOS_INDEX = 0;
    public static final int DEIMOS_INDEX = 1;

    private static Client clientInstance;
    private static Backend backendInstance;

    public static Client getClientInstance() {
        return clientInstance;
    }

    public static void setClientInstance(Client client) {
        clientInstance = client;
    }

    public static Backend getBackendInstance() {
        return backendInstance;
    }

    public static void setBackendInstance(Backend backend) {
        backendInstance = backend;
    }
}

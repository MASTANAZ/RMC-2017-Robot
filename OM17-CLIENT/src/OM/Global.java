package OM;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class Global {
    private static Client clientInstance = null;
    private static boolean running = true;

    public static void setClientInstance(Client client) {
        clientInstance = client;
    }

    public static Client getClientInstance() {
        return clientInstance;
    }

    public static boolean isRunning() {
        return running;
    }

    public static void setRunning(boolean isRunning) {
        running = isRunning;
    }
}

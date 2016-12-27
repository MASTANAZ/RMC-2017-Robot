package OM.Client.Backend;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;

/**
 * Created by Harris on 12/25/16.
 */
public class RNI {
    // network usage in total bytes of data
    private static long received, sent, total;
    private static ServerSocket server = null;

    private static ArrayList<Socket> clients;
    private static ArrayList<InputStream> inputs;
    private static ArrayList<OutputStream> outputs;

    private static final int PORT = 12000;
    private static final int SERVER_TIMEOUT_MS = 10;

    private static SimpleDoubleProperty connectionAProperty = null;
    private static SimpleDoubleProperty connectionBProperty = null;

    private static final byte CB_STATEMENT_END = 0x00;

    public RNI() {

    }

    public static void initialize() {
        clients = new ArrayList<Socket>();
        inputs = new ArrayList<InputStream>();
        outputs = new ArrayList<OutputStream>();

        connectionAProperty = new SimpleDoubleProperty(-1.0f);
        connectionBProperty = new SimpleDoubleProperty(-1.0f);

        System.out.println("> CREATING SERVER AT \"localhost:12000\"");
        try {
            server = new ServerSocket(PORT);
            // 10 millisecond timeout so the rest of the backend thread can run
            server.setSoTimeout(SERVER_TIMEOUT_MS);
        } catch (Exception e){
            e.printStackTrace();
        }
        System.out.println("> SUCCESS!");
    }

    public static void tick(float dt) {
        try {
            Socket tmp = server.accept();
            processClient(tmp);
        } catch (SocketTimeoutException e) {
        } catch (Exception e) { e.printStackTrace(); }
    }

    public static void cleanup() {
        System.out.println("> DESTROYING SERVER AT \"localhost:12000\"");
        try {
            server.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.out.println("> SUCCESS!");
    }

    public static DoubleProperty connectionAProperty() {
        return connectionAProperty;
    }

    public static DoubleProperty connectionBProperty() {
        return connectionBProperty;
    }

    private static void processClient(Socket client) throws Exception {
        System.out.println("> PROCESSING NEW CLIENT CONNECTION");
        InputStreamReader in = new InputStreamReader(client.getInputStream());

        String key = "";

        while (in.ready()) {
            System.out.print((char)in.read());
        }

        //System.out.println(key);
        //client.close();
        System.out.println("> SUCCESSFULLY ADDED NEW CLIENT \"\"");
    }
}

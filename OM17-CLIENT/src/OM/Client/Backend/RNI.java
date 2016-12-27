package OM.Client.Backend;

import javafx.beans.property.*;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by Harris on 12/25/16.
 */
public class RNI {
    // constants
    private static final int PORT = 12000;
    private static final int SERVER_TIMEOUT_MS = 10;

    private static final String CONNECTION_KEY = "MINERS_WIN";

    // network control bytes
    private static final byte CB_END = 0x00;
    private static final byte CB_LCV = 0x01;
    private static final byte CB_RCV = 0x02;
    private static final byte CB_PSX = 0x03;
    private static final byte CB_PSY = 0x04;

    // network variables
    private static long received = 0, sent = 0, total = 0;
    private static ServerSocket server = null;

    private static ArrayList<Socket> clients;
    private static ArrayList<InputStreamReader> inputs;
    private static ArrayList<OutputStream> outputs;
    private static ArrayList<Map> properties;

    // front end variables
    private static SimpleDoubleProperty connectionAProperty = null;
    private static SimpleDoubleProperty connectionBProperty = null;
    private static SimpleStringProperty receivedProperty = null;
    private static SimpleStringProperty sentProperty = null;
    private static SimpleStringProperty totalProperty = null;

    public RNI() {

    }

    public static void initialize() {
        clients = new ArrayList<Socket>();
        inputs = new ArrayList<InputStreamReader>();
        outputs = new ArrayList<OutputStream>();
        properties = new ArrayList<Map>();

        connectionAProperty = new SimpleDoubleProperty(-1.0f);
        connectionBProperty = new SimpleDoubleProperty(-1.0f);
        receivedProperty = new SimpleStringProperty("RECEIVED: 0B");
        sentProperty = new SimpleStringProperty("SENT: 0B");
        totalProperty = new SimpleStringProperty("TOTAL: 0B");

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

        // process network commands for each connected client
        for (int i = 0; i < clients.size(); ++i) {
            try {
                while (inputs.get(i).ready()) {
                    byte cb = fetchByte(inputs.get(i));
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static void synchronizedTick() {
        if (clients.size() > 0) connectionAProperty.set(1.0f);
        if (clients.size() > 1) connectionBProperty.set(1.0f);

        receivedProperty.set("RECEIVED: " + Long.toString(received) + "B");
        sentProperty.set("SENT: " + Long.toString(sent) + "B");
        totalProperty.set("TOTAL: " + Long.toString(received + sent) + "B");
    }

    public static void cleanup() {
        // close all active client connections
        System.out.println("> CLOSING ACTIVE CLIENTS");
        for (Socket c : clients) {
            try {
                c.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        System.out.println("> SUCCESS!");

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

    public static StringProperty receivedProperty() {
        return receivedProperty;
    }

    public static StringProperty sentProperty() {
        return sentProperty;
    }

    public static StringProperty totalProperty() {
        return totalProperty;
    }

    private static void processClient(Socket client) throws Exception {
        System.out.println("> PROCESSING NEW CLIENT CONNECTION");
        InputStreamReader in = new InputStreamReader(client.getInputStream());

        String key = "";

        while (in.ready()) {
            key += (char)fetchByte(in);
        }

        if (key.equals(CONNECTION_KEY)) {
            System.out.println("> CLIENT KEY ACCEPTED");
            System.out.println("> SUCCESSFULLY ADDED NEW CLIENT");

            clients.add(client);
            inputs.add(in);
            outputs.add(client.getOutputStream());
            properties.add(new HashMap());
        } else {
            System.err.println("! ERROR: UNRECOGNIZED CLIENT, TERMINATING CONNECTION");
            client.close();
        }
    }

    private static byte fetchByte(InputStreamReader inputStreamReader) throws Exception {
        ++received;
        return (byte)inputStreamReader.read();
    }
}

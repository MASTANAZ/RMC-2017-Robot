package om.mc.backend;

import om.mc.Global;
import javafx.beans.property.*;

import java.io.*;
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
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static final int PORT              = 12000;
    private static final int SERVER_TIMEOUT_MS = 10;

    private static final String CONNECTION_KEY   = "@";
    private static final String CONFIRMATION_KEY = "!";

    private static final int MAX_CLIENTS = 2;

    public static final int S_END           = 0xFF;

    public static final int S_P_X           = 0x01;
    public static final int S_P_Y           = 0x02;
    public static final int S_P_ORIENTATION = 0x03;

    public static final int S_OBSTACLE      = 0x05;

    private static final int PROPERTY_MAX_SIZE = 4;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static long received = 0, sent = 0;
    private static ServerSocket server = null;
    private static int nClients = 0;

    private static ArrayList<Socket> clients;
    private static ArrayList<DataInputStream> inputs;
    private static ArrayList<DataOutputStream> outputs;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // JAVAFX VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static SimpleDoubleProperty phobosConnProperty = null;
    private static SimpleDoubleProperty deimosConnProperty = null;
    private static SimpleStringProperty receivedProperty = null;
    private static SimpleStringProperty sentProperty = null;
    private static SimpleStringProperty totalProperty = null;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public RNI() {

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static void initialize() {
        clients = new ArrayList<Socket>();
        inputs = new ArrayList<DataInputStream>();
        outputs = new ArrayList<DataOutputStream>();

        phobosConnProperty = new SimpleDoubleProperty(-1.0f);
        deimosConnProperty = new SimpleDoubleProperty(-1.0f);
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
            if (nClients < 2) {
                Socket client = server.accept();
                processClient(client);
            }
        } catch (SocketTimeoutException e) {
        } catch (Exception e) { e.printStackTrace(); }

        // process network commands for each connected client
        for (int i = 0; i < clients.size(); ++i) {
            try {
                if (inputs.get(i).available() > 0) {
                    processStatement(inputs.get(i), i);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static void synchronizedTick() {
        if (clients.size() > 0) phobosConnProperty.set(1.0f);
        if (clients.size() > 1) deimosConnProperty.set(1.0f);

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

        // destroy server and free up port on this computer
        System.out.println("> DESTROYING SERVER AT \"localhost:12000\"");
        try {
            server.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        System.out.println("> SUCCESS!");

        clients.clear();
        outputs.clear();
        inputs.clear();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static DoubleProperty phobosConnProperty() {
        return phobosConnProperty;
    }

    public static DoubleProperty deimosConnProperty() {
        return deimosConnProperty;
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

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static void processClient(Socket client) throws Exception {
        // make sure we aren't already maxed out on clients
        if (nClients == MAX_CLIENTS) {
            System.err.println("! ERROR: ALREADY REACHED THE MAXIMUM NUMBER OF CLIENTS");
            System.out.println("> TERMINATING CONNECTION");
            client.close();
            return;
        }

        System.out.println("> ATTEMPTING TO PROCESS NEW CLIENT");

        DataInputStream in = new DataInputStream(client.getInputStream());
        DataOutputStream out = new DataOutputStream(client.getOutputStream());

        // wait for client to send connection key
        Thread.sleep(250);

        String key = "";

        while (in.available() > 0) {
            key += (char)fetchByte(in);
        }

        if (key.equals(CONNECTION_KEY)) {
            System.out.println("> CONNECTION KEY ACCEPTED");
            System.out.println("> SENDING CONFIRMATION KEY TO CLIENT");

            writeString(out, CONFIRMATION_KEY);

            clients.add(client);
            inputs.add(in);
            outputs.add(out);

            ++nClients;

            System.out.println("> SUCCESSFULLY ADDED NEW CLIENT");
        } else {
            System.err.println("! ERROR: UNRECOGNIZED CLIENT");
            System.out.println("> TERMINATING CONNECTION");
            client.close();
        }
    }

    private static int fetchByte(DataInputStream inputStream) throws Exception {
        ++received;
        return inputStream.readUnsignedByte();
    }

    private static void writeString(DataOutputStream out, String toWrite) throws Exception {
        sent += toWrite.length();
        out.writeBytes(toWrite);
    }

    private static void processStatement(DataInputStream inputStream, int sender) throws Exception {
        int id = fetchByte(inputStream);

        int dataBytes[] = {0, 0, 0, 0};
        int j = 3;

        while (true) {
            int dataByte = fetchByte(inputStream);
            if (dataByte == S_END) {
                break;
            } else {
                dataBytes[j] = dataByte;
                --j;
            }
        }

        // phobos always connects first so we know it will be in the zero position
        DataModel.putData(sender, id, dataBytes);
    }
}

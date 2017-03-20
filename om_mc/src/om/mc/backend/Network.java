package om.mc.backend;

import om.mc.Global;
import om.mc.frontend.ConnectionController;
import om.mc.frontend.NetworkController;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.ArrayList;

/**
 * Created by Harris Newsteder on 3/6/17.
 */

public class Network {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static final int SERVER_PORT         = 12000;
    private static final int SERVER_TIMEOUT_MS   = 10;

    private static final String CONNECTION_KEY   = "@";
    private static final String CONFIRMATION_KEY = "!";

    private static final int MAX_CLIENTS         = 2;

    public static final int S_POSE               = 0x01;
    public static final int S_LCV                = 0x02;
    public static final int S_RCV                = 0x03;
    public static final int S_CELL_COST          = 0x04;
    public static final int S_CPU_TEMP           = 0x05;
    public static final int S_STARTING_PARAMS    = 0x06;
    public static final int S_ROUND_START        = 0x07;
    public static final int S_ROUND_STOP         = 0x08;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static ArrayList<Client> clientList;
    private static ServerSocket server;
    private static NetworkController networkController;
    private static ConnectionController connectionController;

    private static long sent, received, total;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static void initialize() {
        clientList = new ArrayList<Client>();

        sent = 0;
        received = 0;
        total = 0;

        networkController = null;

        // host the server at our current ip on the specified port
        try {
            System.out.println("CREATING SERVER AT \"localhost:" + SERVER_PORT + "\"");
            server = new ServerSocket(SERVER_PORT);
            server.setSoTimeout(SERVER_TIMEOUT_MS);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void tick(float dt) {
        if (clientList.size() < 2) {
            try {
                Socket socket = server.accept();
                processClient(socket);
            } catch (SocketTimeoutException e) {
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        transferIncoming();
    }

    public static void syncTick() {
        if (networkController != null) {
            total = sent + received;

            // update network JavaFX elements
            networkController.sentLabel.setText("SENT: " + sent + "B");
            networkController.recvLabel.setText("RECV: " + received + "B");
            networkController.totlLabel.setText("TOTL: " + total + "B");
        }

        if (connectionController != null) {
            if (clientList.size() >= 1) {
                connectionController.phobosProgress.setProgress(1.0);
            }

            if (clientList.size() >= 2) {
                connectionController.deimosProgress.setProgress(1.0);
            }
        }
    }

    public static void cleanup() {
        // close all active clients
        try {
            System.out.println("CLOSING ALL ACTIVE CLIENT CONNECTIONS");
            for (Client c : clientList) {
                c.socket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        // unbind the server socket
        try {
            System.out.println("DESTROYING SERVER AT \"localhost:" + SERVER_PORT + "\"");
            server.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void bindNetworkController(NetworkController nc) {
        networkController = nc;
    }

    public static void bindConnectionController(ConnectionController cc) {
        connectionController = cc;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static void processClient(Socket candidate) throws Exception {
        if (clientList.size() >= MAX_CLIENTS) {
            System.err.println("ERROR: MAX NUMBER OF CLIENTS REACHED\nTERMINATING CONNECTION");
            candidate.close();
            return;
        }

        System.out.println("PROCESSING NEW CLIENT");

        Client c = new Client();

        c.inStream = new DataInputStream(candidate.getInputStream());
        c.outStream = new DataOutputStream(candidate.getOutputStream());
        c.socket = candidate;

        // wait for client to send connection key
        Thread.sleep(250);

        // get the first byte sent by the client and compare it to the connection key
        String key = "" + (char)readByte(c.inStream);

        // client didn't send the proper connection byte
        if (!key.equals(CONNECTION_KEY)) {
            System.err.println("ERROR: UNRECOGNIZED CLIENT\nTERMINATING CONNECTION");
            candidate.close();
            return;
        }

        // client sent the correct connection key
        System.out.println("CONNECTION KEY ACCEPTED; SENDING CONFIRMATION KEY");
        writeString(c.outStream, CONFIRMATION_KEY);
        System.out.println("CLIENT SUCCESSFULLY ADDED");

        clientList.add(c);
    }

    private static int readByte(DataInputStream in) throws Exception {
        received++;
        return in.readUnsignedByte();
    }

    public static void writeString(DataOutputStream out, String toWrite) throws Exception {
        sent += toWrite.length();
        out.writeBytes(toWrite);
    }

    private static void transferIncoming() {
        for (int i = 0; i < clientList.size(); ++i) {
            DataInputStream in = clientList.get(i).inStream;
            Robot robot = Global.getBackendInstance().getMission().getRobot(i);

            // go through all the bytes in waiting for each client and push them to the statementList for the respective
            // virtual robot representation
            try {
                while (in.available() > 0) {
                    robot.pushStatement(readByte(in));
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static Client getClient(int index) {
        return clientList.get(index);
    }

    public static int getClientSize() {
        return clientList.size();
    }
}

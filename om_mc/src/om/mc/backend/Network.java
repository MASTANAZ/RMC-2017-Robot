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
    public static final int S_MC1                = 0x02;
    public static final int S_MC2                = 0x03;
    public static final int S_CELL_COST          = 0x04;
    public static final int S_CPU_TEMP           = 0x05;
    public static final int S_STARTING_PARAMS    = 0x06;
    public static final int S_ROUND_START        = 0x07;
    public static final int S_ROUND_STOP         = 0x08;
    public static final int S_AUTONOMY_STOP      = 0x09;
    public static final int S_AUTONOMY_START     = 0x0A;
    public static final int S_STATE              = 0x0B;
    public static final int S_CONTROL_STATE      = 0x0C;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private static ArrayList<Client> clientList;
    private static ServerSocket server;
    private static NetworkController networkController;
    private static ConnectionController connectionController;

    private static long sent, received, total;

    private static int nConnected;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static void initialize() {
        clientList = new ArrayList<Client>();

        for (int i = 0; i < MAX_CLIENTS; ++i) {
            clientList.add(new Client());
        }

        sent = 0;
        received = 0;
        total = 0;

        networkController = null;

        nConnected = 0;

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
        if (nConnected < MAX_CLIENTS) {
            try {
                Socket socket = server.accept();
                processClient(socket);
            } catch (SocketTimeoutException e) {
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        monitorClients(dt);
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
            if (clientList.get(0).alive) {
                connectionController.phobosProgress.setProgress(1.0);
            } else {
                connectionController.phobosProgress.setProgress(-1.0);
            }

            if (clientList.get(1).alive) {
                connectionController.deimosProgress.setProgress(1.0);
            } else {
                connectionController.deimosProgress.setProgress(-1.0);
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
        if (nConnected >= MAX_CLIENTS) {
            System.err.println("ERROR: MAX NUMBER OF CLIENTS REACHED\nTERMINATING CONNECTION");
            candidate.close();
            return;
        }

        System.out.println("PROCESSING NEW CLIENT");

        Client c = new Client();

        c.inStream = new DataInputStream(candidate.getInputStream());
        c.outStream = new DataOutputStream(candidate.getOutputStream());
        c.socket = candidate;
        c.connectionTimer = 0.0f;
        c.alive = true;

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

        // the id of the robot that is connecting (0 for phobos and 1 for deimos)
        int id = readByte(c.inStream);

        if (id == 0) {
            System.out.println("PHOBOS CONNECTED");
        } else {
            System.out.println("DEIMOS CONNECTED");
        }

        // client sent the correct connection key
        System.out.println("CONNECTION KEY ACCEPTED; SENDING CONFIRMATION KEY");
        writeString(c.outStream, CONFIRMATION_KEY);
        System.out.println("CLIENT SUCCESSFULLY ADDED");

        // add the client to its respective position in the list
        clientList.set(id, c);

        nConnected++;
    }

    private static int readByte(DataInputStream in) throws Exception {
        received++;
        return in.readUnsignedByte();
    }

    public static void writeString(DataOutputStream out, String toWrite) throws Exception {
        if (out == null) {
            System.err.println("ERROR: FAILED TO SEND DATA TO NULL CLIENT");
        }

        sent += toWrite.length();
        out.writeBytes(toWrite);
    }

    private static void transferIncoming() {
        for (int i = 0; i < clientList.size(); ++i) {
            if (clientList.get(i).alive == false) continue;

            DataInputStream in = clientList.get(i).inStream;
            Robot robot = Global.getBackendInstance().getMission().getRobot(i);

            // go through all the bytes in waiting for each client and push them to the statementList for the respective
            // virtual robot representation
            try {
                while (in.available() > 0) {
                    // we received data from this client, reset the timer so we know the connection is not dead
                    clientList.get(i).connectionTimer = 0.0f;
                    robot.pushStatement(readByte(in));
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private static void monitorClients(float dt)
    {
        for (Client c : clientList) {
            c.connectionTimer += dt;

            if (c.connectionTimer > 5.0f && c.alive) {
                c.alive = false;
                System.err.println("LOST CLIENT CONNECTION");
                nConnected--;
            }
        }
    }

    public static Client getClient(int index) {
        return clientList.get(index);
    }

    public static int getConnected() {
        return nConnected;
    }
}

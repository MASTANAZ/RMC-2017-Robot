package OM;

import javafx.concurrent.Task;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.Socket;

/**
 * Created by OSPREY MINERS on 12/9/2016.
 */
public class RNI {
    private static Socket sockA = null, sockB = null;
    private static DataOutputStream outA = null, outB = null;
    private static DataInputStream inA = null, inB = null;

    public static void initialize() {

    }

    public static boolean connect() {
        try {
            sockA = new Socket("192.168.1.178", 12000);
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        try {
            outA = new DataOutputStream(sockA.getOutputStream());
            inA = new DataInputStream(sockA.getInputStream());
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        try {
            outA.writeChars("test...");
        } catch (Exception e) {
            e.printStackTrace();
        }

        Task task = new Task<Void>() {
            @Override public Void call() {
                while (Global.isRunning()) {
                    try {
                        String pos = "";

                        while (inA.available() > 0) {
                            char c = (char)inA.read();
                            if (c == '\n') {
                                Round.getRobotA().setPosition(Float.parseFloat(pos), 1.5f);
                                break;
                            } else {
                                pos += c;
                            }
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
                return null;
            }
        };

        new Thread(task).start();

        return true;
    }

    public static void cleanup() {
        if (sockA != null) {
            try {
                sockA.close();
                outA.close();
                inA.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}

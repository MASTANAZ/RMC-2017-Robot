package OM17;

import java.io.*;
import java.net.*;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class RNI {
    public static Socket socketA, socketB;
    public static DataOutputStream outA, outB;

    public static boolean connect(String addressA, String addressB) {
        // parse addressA and addressB from their respective strings
        try {
            socketA = new Socket("192.168.1.178", 12000);
            outA = new DataOutputStream(socketA.getOutputStream());
            outA.writeBytes("fuck you 'bama");
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        return true;
    }

    public static void cleanup() {
        if (socketA != null) {
            try {
                socketA.close();
            } catch(Exception e) {
                e.printStackTrace();
            }
        }
    }
}

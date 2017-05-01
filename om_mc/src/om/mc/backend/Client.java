package om.mc.backend;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.Socket;

/**
 * Created by Harris Newsteder on 3/6/17.
 */
public class Client {
    public DataInputStream inStream = null;
    public DataOutputStream outStream = null;
    public Socket socket = null;
    public float connectionTimer = 0.0f;
    public boolean alive = false;
}

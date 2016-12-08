package OM;

/**
 * Created by OSPREY MINERS on 12/8/2016.
 */
public class Round {
    private static Robot a, b;

    public static void initialize() {
        a = new Robot();
        b = new Robot();
    }

    public static Robot getRobotA() {
        return a;
    }

    public static Robot getRobotB() {
        return b;
    }
}

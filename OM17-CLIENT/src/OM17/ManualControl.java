package OM17;

import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;

/**
 * Created by OSPREY MINERS on 12/2/2016.
 */
public class ManualControl {
    Controller joystick;

    public ManualControl() {
        Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
        for (int i = 0; i < controllers.length; ++i) {
            System.out.println(controllers[i].getName());
            if (controllers[i].getType() == Controller.Type.STICK) {
                joystick = controllers[i];
                break;
            }
        }
    }
}

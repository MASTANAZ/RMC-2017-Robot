package om.mc.backend;

import net.java.games.input.*;
import om.mc.Global;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by Harris Newsteder on 3/10/2017.
 */
public class ManualControl {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static final float DEADZONE_THRESHOLD = 0.2f;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static ArrayList<Controller> controllers;

    private static EventQueue eventQueue;
    private static Event event;

    private static Map controllerValues[];

    private static boolean controllerAvailable;

    private static boolean removeme = false;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static void initialize() {
        controllers = new ArrayList<Controller>();

        eventQueue = null;
        event = new Event();

        controllerValues = null;

        controllerAvailable = false;
    }

    public static void findControllers() {
        // remove all controllers currently in use by the program
        controllerAvailable = false;
        controllers.clear();

        Controller available[];

        // get available controllers from the system
        try {
            available = createDefaultEnvironment().getControllers();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        // look for controllers of type GAMEPAD and add all to our controllers array list
        for (Controller c : available) {
            if (c.getType() == Controller.Type.GAMEPAD) {
                // we've found at least one suitable controller
                controllerAvailable = true;
                controllers.add(c);

                System.out.println("FOUND CONTROLLER: \"" + c.getName() + "\"");
            }
        }

        // create a hash map of values for each available controller (if possible)
        if (!controllerAvailable) {
            System.err.println("ERROR: NO SUITABLE CONTROLLERS FOUND");
            return;
        } else {
            controllerValues = new HashMap[controllers.size()];
        }

        // set all controller component values to 0.0f
        for (int i = 0; i < controllers.size(); ++i) {
            controllerValues[i] = new HashMap();
            for (Component c : controllers.get(i).getComponents()) {
                controllerValues[i].put(c.getIdentifier().getName(), 0.0f);
            }
        }
    }

    public static void tick(float dt) {
        if (!controllerAvailable) return;

        // update the values for every available controller
        for (int i = 0; i < controllers.size(); ++i) {
            controllers.get(i).poll();

            eventQueue = controllers.get(i).getEventQueue();

            while (eventQueue.getNextEvent(event)) {
                float val = event.getValue();

                // set the value to zero if the component is in the dead zone
                if (Math.abs(val) < DEADZONE_THRESHOLD) val = 0.0f;

                controllerValues[i].put(event.getComponent().getIdentifier().getName(), val);
            }

            // we can only control two robots at a time, even if we have more than two controllers
            if (i > 1) return;

            Robot bound = Global.getBackendInstance().getMission().getRobot(i);

            // TODO: FIX

            // AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
            double lcv = (double)((float)controllerValues[i].get("y")) * -100;
            double rcv = (double)((float)controllerValues[i].get("y")) * -100;

            if ((float)controllerValues[i].get("3") == 1.0f && !removeme) {
                removeme = true;
                // asdf
                bound.toggleControlState();
            }

            if ((float)controllerValues[i].get("3") == 0.0f) {
                removeme = false;
            }

            double steering = (double)((float)controllerValues[i].get("rx")) * 100;

            lcv += steering;
            rcv -= steering;

            if (lcv > 100) lcv = 100;
            if (lcv < -100) lcv = -100;

            if (rcv > 100) rcv = 100;
            if (rcv < -100) rcv = -100;

            bound.setLCV((int)lcv);
            bound.setRCV((int)rcv);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static ControllerEnvironment createDefaultEnvironment() throws ReflectiveOperationException {
        Constructor<ControllerEnvironment> constructor = (Constructor<ControllerEnvironment>)
                Class.forName("net.java.games.input.DefaultControllerEnvironment").getDeclaredConstructors()[0];

        constructor.setAccessible(true);

        return constructor.newInstance();
    }
}

package OM.MC.Backend;

import OM.MC.Global;
import net.java.games.input.*;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by Harris on 12/25/16.
 */
public class ManualControl {
    private final float DEADZONE_THRESHOLD = 0.2f;

    private ArrayList<Controller> controllers = null;
    private EventQueue eventQueue = null;
    private Event event = null;
    private Map controllerValues[] = null;
    private boolean controllerAvailable = false;

    public ManualControl() {
        controllers = new ArrayList<Controller>();
        event = new Event();
    }

    public void findControllers() {
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

                System.out.println("> FOUND CONTROLLER: \"" + c.getName() + "\"");
            }
        }

        // create a hash map of values for each available controller (if possible)
        if (!controllerAvailable) {
            System.err.println("! ERROR: NO SUITABLE CONTROLLERS FOUND");
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

    public void tick(float dt) {
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
        }
    }

    private static ControllerEnvironment createDefaultEnvironment() throws ReflectiveOperationException {
        Constructor<ControllerEnvironment> constructor = (Constructor<ControllerEnvironment>)
                Class.forName("net.java.games.input.DefaultControllerEnvironment").getDeclaredConstructors()[0];

        constructor.setAccessible(true);

        return constructor.newInstance();
    }
}

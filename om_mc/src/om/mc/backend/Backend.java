package om.mc.backend;

import javafx.application.Platform;
import javafx.concurrent.Task;

/**
 * Created by Harris Newsteder on 3/6/17.
 */
public class Backend {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private boolean running;

    private long oldTime, newTime;
    private float dt;

    private Mission mission;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Backend() {
        running = true;
        oldTime = 0;
        newTime = 0;
        dt = 0.0f;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void start() {
        initialize();

        Task backendTask = new Task<Void>() {
            @Override
            public Void call() {
                // this ensures our first value for dt is zero (or close enough)
                oldTime = System.nanoTime();

                while (running) {
                    // delta time calculation
                    // delta time is the time (in seconds) since the last tick on this thread, it helps with all timing
                    // on the backend thread and with time-dependent tasks (such as animations)
                    newTime = System.nanoTime();
                    dt = (float)(newTime - oldTime) / 1000000000.0f;
                    oldTime = newTime;

                    tick(dt);

                    // all parts of the backend that need to update elements or be synchronized with the JavaFX thread
                    Platform.runLater(new Runnable() {
                        @Override
                        public void run() {
                            syncTick();
                        }
                    });

                    // give the CPU time to perform other tasks
                    try {
                        Thread.sleep(1);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                cleanup();

                return null;
            }
        };

        new Thread(backendTask).start();
    }

    public void stop() {
        running = false;
    }

    public void cleanup() {
        Network.cleanup();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void initialize() {
        Network.initialize();
        ManualControl.initialize();

        // the Network needs to initialize before the mission does
        mission = new Mission();
    }

    private void tick(float dt) {
        Network.tick(dt);
        ManualControl.tick(dt);
        mission.tick(dt);
    }

    private void syncTick() {
        Network.syncTick();
        mission.syncTick();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETTERS / GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Mission getMission() {
        return mission;
    }
}

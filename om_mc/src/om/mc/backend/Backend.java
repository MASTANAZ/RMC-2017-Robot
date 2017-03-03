package om.mc.backend;

import om.mc.Global;
import javafx.application.Platform;
import javafx.concurrent.Task;

/**
 * Created by Harris on 12/25/16.
 */
public class Backend {
    private boolean running = false;
    private long oldTime = 0;
    private long currentTime = 0;
    private float dt = 0.0f;

    private Mission mission;
    private ManualControl manualControl;

    public Backend() {
        mission = new Mission();
        manualControl = new ManualControl();
        Global.setBackendInstance(this);
    }

    public void initialize() {
        Network.initialize();
        DataModel.initialize();
    }

    public void start() {
        running = true;

        // the backend thread
        Task task = new Task<Void>() {
            @Override
            public Void call() {
                oldTime = System.nanoTime();

                while (running) {
                    // delta time calculation
                    currentTime = System.nanoTime();
                    dt = (float)(currentTime - oldTime) / 1000000000.0f;

                    tick();

                    // allows all GUI elements to be updated on the JavaFX thread
                    Platform.runLater(new Runnable() {
                        @Override
                        public void run() {
                            synchronizedTick();
                        }
                    });

                    // give the CPU some time to do other tasks
                    try {
                        Thread.sleep(1);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    oldTime = currentTime;
                }

                cleanup();

                return null;
            }
        };

        new Thread(task).start();
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        running = false;
    }

    public Mission getMission() {
        return mission;
    }

    public ManualControl getManualControl() {
        return manualControl;
    }

    private void tick() {
        Network.tick(dt);
        DataModel.tick(dt);
        manualControl.tick(dt);
        mission.tick(dt);
    }

    private void synchronizedTick() {
        Network.synchronizedTick();
        mission.synchronizedTick();
    }

    private void cleanup() {
        Network.cleanup();
    }
}

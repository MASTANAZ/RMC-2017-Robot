package OM.Client.Backend;

import OM.Client.Global;
import javafx.application.Platform;
import javafx.concurrent.Task;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.transform.Affine;

/**
 * Created by Harris on 12/25/16.
 */
public class Backend {
    private boolean running = false;
    private long oldTime = 0;
    private long currentTime = 0;
    private float dt = 0.0f;
    private Canvas canvas = null;
    private GraphicsContext gc = null;

    private Mission mission;
    private ManualControl manualControl;

    public Backend() {
        mission = new Mission();
        manualControl = new ManualControl();
        Global.setBackendInstance(this);
    }

    public void start() {
        running = true;

        manualControl.findControllers();

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

    public void stop() {
        running = false;
        mission.stop();
    }

    public void setCanvas(Canvas canvas) {
        this.canvas = canvas;
        gc = canvas.getGraphicsContext2D();

    }

    public Mission getMission() {
        return mission;
    }

    private void tick() {
        RNI.tick(dt);
        manualControl.tick(dt);
        mission.tick(dt);
    }

    private void synchronizedTick() {
        RNI.synchronizedTick();
        mission.synchronizedTick();
    }

    private void cleanup() {
        RNI.cleanup();
    }
}

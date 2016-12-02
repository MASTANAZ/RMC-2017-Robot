package OM17;

import javafx.concurrent.Task;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class FieldPane extends Canvas {
    private GraphicsContext gc;

    private final static int TARGET_WIDTH = 1280;
    private final static int TARGET_HEIGHT = 720;

    private Robot a;

    public FieldPane(Client client) {
        gc = this.getGraphicsContext2D();

        a = new Robot();

        Task task = new Task<Void>() {
            @Override public Void call() {
                while (client.getPrimaryStage().isShowing()) {

                    updateField();
                    drawField(client);

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
    }

    private void updateField() {
        // fetch data from RNI and update values
    }

    private void drawField(Client client) {
        // resize drawing box based on current window size
        setWidth(client.getPrimaryStage().getScene().getWidth() / 2.0f);
        setHeight(client.getPrimaryStage().getScene().getHeight() / 2.0f);

        gc.clearRect(0, 0, getWidth(), getHeight());

        // scaling the graphics to display properly inside the drawing box
        gc.scale((double)getWidth() / (double)TARGET_WIDTH, (double)getHeight() / (double)TARGET_HEIGHT);

        // draw base color
        gc.setFill(Color.SANDYBROWN);
        gc.fillRect(0,0, TARGET_WIDTH, TARGET_HEIGHT);

        a.draw(gc);

        // pop graphics transform matrix
        gc.setTransform(1, 0, 0, 1, 0, 0);
    }
}

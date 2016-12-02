package OM17;

import javafx.concurrent.Task;
import javafx.geometry.Pos;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class FieldPane extends GridPane {
    private GraphicsContext gc;
    private Canvas canvas;

    private final static int TARGET_WIDTH = 640;
    private final static int TARGET_HEIGHT = 360;

    private Robot a;

    public FieldPane(Client client) {
        canvas = new Canvas();
        canvas.setWidth(TARGET_WIDTH);
        canvas.setHeight(TARGET_HEIGHT);

        gc = canvas.getGraphicsContext2D();

        getChildren().add(canvas);

        setAlignment(Pos.TOP_RIGHT);

        a = new Robot();

        Task task = new Task<Void>() {
            @Override public Void call() {
                while (client.getPrimaryStage().isShowing()) {

                    updateField();
                    drawField();

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
        a.update();
    }

    private void drawField() {
        gc.clearRect(0, 0, TARGET_WIDTH, TARGET_HEIGHT);

        // draw base color
        gc.setFill(Color.SANDYBROWN);
        gc.fillRect(0,0, TARGET_WIDTH, TARGET_HEIGHT);

        a.draw(gc);

        // pop graphics transform matrix
        gc.setTransform(1, 0, 0, 1, 0, 0);
    }
}

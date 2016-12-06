package OM;

import javafx.concurrent.Task;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class FieldCanvas extends Canvas {
    private GraphicsContext gc;
    public FieldCanvas(Pane parent) {
        widthProperty().bind(parent.widthProperty());
        heightProperty().bind(parent.heightProperty());

        gc = this.getGraphicsContext2D();

        Task task = new Task<Void>() {
            @Override public Void call() {
                while (Global.isRunning()) {

                    update();
                    draw();

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

    private void draw() {
        gc.setFill(Color.BISQUE);
        gc.fillRect(0, 0, getWidth(), getHeight());
    }

    private void update() {

    }
}

package OM;

import javafx.application.Platform;
import javafx.concurrent.Task;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Created by OSPREY MINERS on 12/6/2016.
 */
public class FieldCanvas extends Canvas {
    private GraphicsContext gc;

    private final float TARGET_WIDTH = 1280.0f;
    private final float TARGET_HEIGHT = 720.0f;

    private final Color COLOR_REGOLITH = new Color(221.0f / 255.0f, 176.0f / 255.0f, 128.0f / 255.0f, 1.0f);
    private final Color COLOR_OBSTACLE_AREA = new Color(221.0f / 255.0f, 158.0f / 255.0f, 128.0f / 255.0f, 1.0f);

    float rot = 0;

    public FieldCanvas(Pane parent) {
        widthProperty().bind(parent.widthProperty());
        heightProperty().bind(parent.heightProperty());

        gc = this.getGraphicsContext2D();
        gc.setLineWidth(0.05);

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
        float scaleX = (float)getWidth() / TARGET_WIDTH;
        float scaleY = (float)getHeight() / TARGET_HEIGHT;

        Affine noTransform = gc.getTransform();

        gc.scale(scaleX, scaleY);

        // clear all graphics from the pane
        gc.clearRect(0, 0, TARGET_WIDTH, TARGET_HEIGHT);

        // for centering our drawn field inside the pane we reside in
        gc.translate(10, 37.32);

        // scale factor for real world meters to on screen pixels
        gc.scale(170.732, 170.732);

        // field base
        gc.setFill(COLOR_REGOLITH);
        gc.fillRect(0, 0, 7.38, 3.78);

        // obstacle field
        gc.setFill(COLOR_OBSTACLE_AREA);
        gc.fillRect(1.5, 0, 2.94, 3.78);

        Round.getRobotA().draw(gc);
        //Round.getRobotB().draw(gc);

        gc.setTransform(noTransform);
    }

    private void update() {
        rot = rot + 1;
    }
}

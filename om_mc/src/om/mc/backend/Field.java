package om.mc.backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Created by Harris Newsteder on 3/7/17.
 */
public class Field {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // the dimensions of the field in meters
    public static final float FIELD_WIDTH    = 7.38f;
    public static final float FIELD_HEIGHT   = 3.78f;

    // the dimensions of the cost map in number of cells
    private static final int GRID_WIDTH       = 48;
    private static final int GRID_HEIGHT      = GRID_WIDTH / 2;

    private static final float CELL_SIZE      = FIELD_WIDTH / (float)GRID_WIDTH;

    private final Color COLOR_REGOLITH        = Color.web("#FFCC80");
    private final Color COLOR_OBSTACLE_AREA   = Color.web("#F2C06F");
    private final Color COLOR_NON_TRAVERSABLE = Color.web("#EE42F4");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VARIABLES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private float costMap[][];

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTRUCTOR
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Field() {
        costMap = new float[GRID_HEIGHT][GRID_WIDTH];

        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                costMap[y][x] = 0.0f;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {

    }

    public void draw(GraphicsContext gc) {
        // grab our graphics transformation matrix before we draw so we can reset it when we're done
        Affine stack = gc.getTransform();

        // drawing the field base
        gc.setFill(COLOR_REGOLITH);
        gc.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);
        // drawing the obstacle area
        gc.setFill(COLOR_OBSTACLE_AREA);
        gc.fillRect(1.5, 0, 2.94, FIELD_HEIGHT);

        // TODO: asdfasdfasdf
        gc.setLineWidth(0.01f);
        gc.setStroke(Color.BLACK);
        gc.strokeLine(0, 1.89f, FIELD_WIDTH, 1.89f);
        gc.strokeLine(1.5f, 0.0f, 1.5f, 3.78f);
        gc.setLineWidth(0.001f);

        gc.translate(0.0f, Field.FIELD_HEIGHT);

        // draw the field representation as seen by the robots (cost map)
        gc.setFill(COLOR_NON_TRAVERSABLE);
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                gc.strokeRect(x * CELL_SIZE, (y * -CELL_SIZE) - CELL_SIZE, CELL_SIZE, CELL_SIZE);
                if (costMap[y][x] < 0.0f) {
                    gc.fillRect(x * CELL_SIZE, (y * -CELL_SIZE) - CELL_SIZE, CELL_SIZE, CELL_SIZE);
                }
            }
        }

        gc.setTransform(stack);
    }

    public void updateCostMap(int x, int y, float cost) {
        costMap[y][x] = cost;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

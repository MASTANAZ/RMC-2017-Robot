package om.mc.backend;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;

/**
 * Created by Harris Newsteder on 3/7/17.
 */
public class Field {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // the dimensions of the field in meters
    private static final float FIELD_WIDTH    = 7.38f;
    private static final float FIELD_HEIGHT   = 3.78f;

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

        costMap[5][5] = -1.0f;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUBLIC FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void tick(float dt) {

    }

    public void draw(GraphicsContext gc) {
        // drawing the field base
        gc.setFill(COLOR_REGOLITH);
        gc.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);
        // drawing the obstacle area
        gc.setFill(COLOR_OBSTACLE_AREA);
        gc.fillRect(1.5, 0, 2.94, FIELD_HEIGHT);

        // draw the field representation as seen by the robots (cost map)
        gc.setFill(COLOR_NON_TRAVERSABLE);
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                //gc.strokeRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
                if (costMap[y][x] < 0.0f) {
                    gc.fillRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
                }
            }
        }
    }

    public void updateCostMap(int x, int y, float cost) {
        costMap[y][x] = cost;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRIVATE FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

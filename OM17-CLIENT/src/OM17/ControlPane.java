package OM17;

import javafx.scene.control.Label;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.RowConstraints;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class ControlPane extends GridPane {
    public ControlPane(Client client) {
        ColumnConstraints c0 = new ColumnConstraints();
        ColumnConstraints c1 = new ColumnConstraints();

        RowConstraints r0 = new RowConstraints();
        RowConstraints r1 = new RowConstraints();

        c0.setPercentWidth(50);
        c1.setPercentWidth(50);

        r0.setPercentHeight(50);
        r1.setPercentHeight(50);

        getColumnConstraints().addAll(c0, c1);
        getRowConstraints().addAll(r0, r1);

        setGridLinesVisible(true);

        add(new FieldPane(client), 0, 1);
    }
}

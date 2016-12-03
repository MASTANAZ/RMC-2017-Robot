package OM17;

import javafx.scene.layout.*;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class MissionPane extends GridPane {
    ManualControl mc;

    public MissionPane(Client client) {
        mc = new ManualControl();

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

        add(new ControlPane(), 0, 0);
        add(new FieldPane(client), 0, 1);
        add(new StatusPane(), 1, 1);
    }


}

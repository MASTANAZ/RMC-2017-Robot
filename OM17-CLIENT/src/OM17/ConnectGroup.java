package OM17;

import javafx.geometry.Pos;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class ConnectGroup extends BorderPane {
    private VBox vbox;

    private TextField addressFieldA, addressFieldB;
    private Label fieldALabel, fieldBLabel;

    private Button connectButton;

    public ConnectGroup() {
        vbox = new VBox();

        fieldALabel = new Label("ADDRESS A");
        fieldBLabel = new Label("ADDRESS B");

        addressFieldA = new TextField("192.168.0.254:25565");
        addressFieldA.setAlignment(Pos.CENTER);
        addressFieldA.setMaxWidth(150);
        addressFieldB = new TextField("192.168.0.255:25565");
        addressFieldB.setAlignment(Pos.CENTER);
        addressFieldB.setMaxWidth(150);

        connectButton = new Button();
        connectButton.setText("CONNECT");

        vbox.setSpacing(5.0);
        vbox.getChildren().addAll(fieldALabel, addressFieldA, fieldBLabel, addressFieldB, connectButton);

        vbox.setAlignment(Pos.CENTER);
        setCenter(vbox);
    }
}

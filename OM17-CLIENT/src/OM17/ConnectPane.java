package OM17;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.VBox;

/**
 * Created by OSPREY MINERS on 11/19/2016.
 */
public class ConnectPane extends BorderPane {
    private ImageView logoView;

    private VBox vbox;

    private TextField addressFieldA, addressFieldB;
    private Label fieldALabel, fieldBLabel;

    private Button connectButton;

    public ConnectPane(Client client) {
        vbox = new VBox();

        try {
            Image logo = new Image("res/connect_screen_logo.png");
            logoView = new ImageView();
            logoView.setImage(logo);
            logoView.setPreserveRatio(true);
        } catch (Exception e) {
            e.printStackTrace();
        }

        fieldALabel = new Label("ADDRESS - ROBOT A");
        fieldBLabel = new Label("ADDRESS - ROBOT B");

        addressFieldA = new TextField("192.168.0.254:25565");
        addressFieldA.setAlignment(Pos.CENTER);
        addressFieldA.setMaxWidth(200);
        addressFieldB = new TextField("192.168.0.255:25565");
        addressFieldB.setAlignment(Pos.CENTER);
        addressFieldB.setMaxWidth(200);

        connectButton = new Button();
        connectButton.setText("CONNECT");

        connectButton.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                // connect through RNI
                client.changeState(client.STATE_CONTROL);
            }
        });

        vbox.setSpacing(5.0);
        vbox.getChildren().addAll(logoView, fieldALabel, addressFieldA, fieldBLabel, addressFieldB, connectButton);
        vbox.setAlignment(Pos.CENTER);

        setCenter(vbox);
    }
}

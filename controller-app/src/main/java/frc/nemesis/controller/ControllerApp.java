package frc.nemesis.controller;

import java.util.HashMap;
import java.util.Map;
import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.stage.Screen;
import javafx.stage.Stage;

public class ControllerApp extends Application {

  private static final String DEFAULT_BUTTON_STYLE = null;
  private static final String SELECTED_BUTTON_STYLE = "-fx-background-color: green";

  private final NetworkTableClient client = NetworkTableClient.getInstance();

  private final Map<String, Button> buttonMap = new HashMap<>();

  private static final String[] compassPoints = {"E", "SE", "S", "SW", "W", "NW", "N", "NE"};

  @Override
  public void start(Stage primaryStage) {
    primaryStage.setTitle("Nemesis Controller");

    // if there are multiple monitors (screens), get the second one, otherwise get the primary one
    var screens = Screen.getScreens();
    var screen = screens.size() > 1 ? screens.get(1) : Screen.getPrimary();
    var bounds = screen.getVisualBounds();

    // Create a BorderPane to hold the content
    BorderPane root = new BorderPane();
    root.setPadding(new Insets(20));

    EventHandler<ActionEvent> buttonHandler = event -> onButtonPress(event);

    // Create a Pane for the compass buttons
    Pane compassPane = new Pane();

    // Define the center coordinates and radius
    double centerX = bounds.getWidth() / 2;
    double centerY = bounds.getHeight() / 2;
    double radius = 200;

    // Create and position compass buttons
    double angleStep = Math.PI / 4; // 45 degrees in radians
    for (int i = 0; i < compassPoints.length; i++) {
      double angle = i * angleStep;
      double x = centerX + radius * Math.cos(angle);
      double y = centerY + radius * Math.sin(angle);

      Button button = new Button(compassPoints[i]);
      buttonMap.put(compassPoints[i], button);

      button.setOnAction(buttonHandler);

      button.setPrefWidth(80);
      button.setPrefHeight(40);
      button.setLayoutX(x - button.getWidth() / 2);
      button.setLayoutY(y - button.getHeight() / 2);
      button.setStyle(DEFAULT_BUTTON_STYLE);
      compassPane.getChildren().add(button);
    }

    // Create an HBox for the bottom buttons
    HBox buttonBox = new HBox(10); // Spacing of 10 between buttons
    buttonBox.setAlignment(Pos.CENTER); // Center the buttons horizontally

    Button connectButton = new Button("Connect");
    connectButton.setOnAction(event -> connect());

    Button refreshButton = new Button("Refresh");
    refreshButton.setOnAction(event -> refresh());

    buttonBox.getChildren().addAll(connectButton, refreshButton);

    // Add the compassPane and buttonBox to the BorderPane
    root.setCenter(compassPane);
    root.setBottom(buttonBox);

    // Create the Scene and set it to the stage
    Scene scene = new Scene(root, bounds.getWidth(), bounds.getHeight());
    primaryStage.setScene(scene);
    primaryStage.setX(bounds.getMinX());
    primaryStage.setY(bounds.getMinY());
    primaryStage.setFullScreen(true);

    // Add the on close handler
    primaryStage.setOnCloseRequest(
        event -> {
          System.out.println("Exiting ...");
          client.disconnect();
        });

    primaryStage.show();

    refresh();
  }

  private void onButtonPress(ActionEvent event) {
    if (!(event.getSource() instanceof Button)) {
      return;
    }
    Button button = (Button) event.getSource();
    String buttonText = button.getText();
    System.out.println("Button Pressed:" + buttonText);
    client.publish("moveTo", buttonText);
    refresh();
  }

  private void refresh() {
    String moveTo = client.getValue("moveTo");
    System.out.println("Refresh Pressed: " + moveTo);

    for (Map.Entry<String, Button> entry : buttonMap.entrySet()) {
      Button button = entry.getValue();
      if (entry.getKey().equals(moveTo)) {
        button.setStyle(SELECTED_BUTTON_STYLE);
      } else {
        button.setStyle(DEFAULT_BUTTON_STYLE);
      }
    }
  }

  private void connect() {
    client.connect();
  }

  public static void main(String[] args) {
    launch(args);
  }
}

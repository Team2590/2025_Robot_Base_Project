package frc.nemesis.controller;

import java.util.HashMap;
import java.util.Map;
import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;

public class ControllerApp extends Application {

  private static final String DEFAULT_BUTTON_STYLE = null;
  private static final String SELECTED_BUTTON_STYLE = "-fx-background-color: green";

  private final NetworkTableClient client = NetworkTableClient.getInstance();

  private final Map<String, ToggleButton> buttonMap = new HashMap<>();

  private static final String[] compassPoints = {"S", "SW", "SE", "N", "NW", "NE"};

  public void start(Stage primaryStage) {
    primaryStage.setTitle("Nemesis Controller");

    BorderPane root = new BorderPane();
    root.setPadding(new Insets(20));

    StackPane stackPane = new StackPane();

    double radius = 200;    
    double centerX = 0;
    double centerY = 0;

    // Create and position compass buttons
    double angleStep = Math.PI / 3; // 60 degrees in radians
    double startAngle = Math.PI / 2; // 30 degrees to make the bottom flat
    
    for (int i = 0; i < compassPoints.length; i++) {
        double angle = i * angleStep + startAngle;
        double x = centerX + radius * Math.cos(angle);
        double y = centerY + radius * Math.sin(angle);
  
        ToggleButton button = new ToggleButton(compassPoints[i]);
        buttonMap.put(compassPoints[i], button);
    
        button.setPrefWidth(80);
        button.setPrefHeight(40);
        
        button.setTranslateX(x);
        button.setTranslateY(y);
        
        button.setStyle(DEFAULT_BUTTON_STYLE);
        stackPane.getChildren().add(button);
    }

    // Add the stackPane to the center of the BorderPane
    root.setCenter(stackPane);

    // Create the Scene and set it to the stage
    Scene scene = new Scene(root, 600, 600);
    scene.getStylesheets().add(getClass().getResource("/styles.css").toExternalForm());
    primaryStage.setScene(scene);
    primaryStage.setFullScreen(true);

    buttonMap.forEach((String point, ToggleButton btn) -> btn.getStyleClass().add("button"));

    // Add the on close handler
    primaryStage.setOnCloseRequest(event -> {
        System.out.println("Exiting ...");
        client.disconnect();
    });

    primaryStage.show();
  }

  public static void main(String[] args) {
    launch(args);
  }
}

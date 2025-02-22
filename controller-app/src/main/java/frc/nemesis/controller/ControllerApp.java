package frc.nemesis.controller;

import java.util.HashMap;
import java.util.Map;
import javafx.animation.FadeTransition;
import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.util.Duration;

public class ControllerApp extends Application {

  private static final String DEFAULT_BUTTON_STYLE = null;
  private static final String SELECTED_BUTTON_STYLE = "-fx-background-color: green";
  private static final String LEVEL_BUTTON_STYLE = "-fx-background-color: lightblue";
  private static final String SELECTED_LEVEL_STYLE = "-fx-background-color: blue";
  private static final String SIDE_BUTTON_STYLE = "-fx-background-color: lightgray";
  private static final String SELECTED_SIDE_STYLE = "-fx-background-color: darkgray";
  private static final String GO_BUTTON_STYLE =
      "-fx-background-color: #90EE90; -fx-font-size: 18; -fx-font-weight: bold";
  private static final String GO_BUTTON_PRESSED_STYLE =
      "-fx-background-color: #32CD32; -fx-font-size: 18; -fx-font-weight: bold";

  private final NetworkTableClient client = NetworkTableClient.getInstance();
  private final Map<String, Button> buttonMap = new HashMap<>();
  private final Map<String, Button> levelButtonMap = new HashMap<>();
  private ToggleButton leftButton;
  private ToggleButton rightButton;
  private Button goButton;
  private String pendingCommand = null;

  private String selectedDirection = null;
  private String selectedSide = "left";
  private String selectedLevel = "L4";

  private static final String[] compassPoints = {"S", "SW", "NW", "N", "NE", "SE"};
  private static final String[] levels = {"L1", "L2", "L3", "L4"};

  private VBox selectionBox;
  private Pane mainPane;

  @Override
  public void start(Stage primaryStage) {
    primaryStage.setTitle("Nemesis Controller");

    // Create a BorderPane to hold the content
    BorderPane root = new BorderPane();
    root.setPadding(new Insets(20));

    mainPane = new Pane();
    createCompassButtons();

    selectionBox = createSelectionBox();
    selectionBox.setVisible(false);
    mainPane.getChildren().add(selectionBox);

    HBox buttonBox = new HBox(10);
    buttonBox.setAlignment(Pos.CENTER);

    Button connectButton = new Button("Connect");
    connectButton.setOnAction(event -> connect());

    Button refreshButton = new Button("Refresh");
    refreshButton.setOnAction(event -> refresh());

    buttonBox.getChildren().addAll(connectButton, refreshButton);

    root.setCenter(mainPane);
    root.setBottom(buttonBox);

    Scene scene = new Scene(root, 800, 600);
    primaryStage.setScene(scene);
    primaryStage.setFullScreen(true);

    primaryStage.setOnCloseRequest(
        event -> {
          System.out.println("Exiting ...");
          client.disconnect();
        });

    primaryStage.show();
    refresh();
  }

  private void createCompassButtons() {
    EventHandler<ActionEvent> buttonHandler = event -> onButtonPress(event);

    double centerX = 400;
    double centerY = 300;
    double radius = 200;
    double angleStep = Math.PI / 3;
    double startAngle = Math.PI / 2;

    for (int i = 0; i < compassPoints.length; i++) {
      double angle = i * angleStep + startAngle;
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
      mainPane.getChildren().add(button);
    }
  }

  private VBox createSelectionBox() {
    VBox box = new VBox(10);
    box.setPadding(new Insets(10));
    box.setStyle("-fx-background-color: white; -fx-border-color: gray; -fx-border-width: 1;");

    // Side selection
    HBox sideBox = new HBox(10);
    leftButton = new ToggleButton("Left");
    rightButton = new ToggleButton("Right");

    leftButton.setOnAction(
        e -> {
          selectedSide = "left";
          leftButton.setStyle(SELECTED_SIDE_STYLE);
          rightButton.setStyle(SIDE_BUTTON_STYLE);
          updatePendingCommand();
        });

    rightButton.setOnAction(
        e -> {
          selectedSide = "right";
          rightButton.setStyle(SELECTED_SIDE_STYLE);
          leftButton.setStyle(SIDE_BUTTON_STYLE);
          updatePendingCommand();
        });

    sideBox.getChildren().addAll(leftButton, rightButton);
    sideBox.setAlignment(Pos.CENTER);

    // Level selection
    HBox levelBox = new HBox(10);
    for (String level : levels) {
      Button levelButton = new Button(level);
      levelButtonMap.put(level, levelButton);

      levelButton.setOnAction(
          e -> {
            selectedLevel = level;
            updateLevelButtons();
            updatePendingCommand();
          });

      levelBox.getChildren().add(levelButton);
    }
    levelBox.setAlignment(Pos.CENTER);

    // GO button
    goButton = new Button("GO");
    goButton.setPrefWidth(100);
    goButton.setPrefHeight(40);
    goButton.setStyle(GO_BUTTON_STYLE);
    goButton.setOnAction(e -> sendToNetworkTables());

    box.getChildren().addAll(sideBox, levelBox, goButton);
    box.setAlignment(Pos.CENTER);
    return box;
  }

  private void resetSelections() {
    selectedSide = "left";
    leftButton.setStyle(SELECTED_SIDE_STYLE);
    rightButton.setStyle(SIDE_BUTTON_STYLE);

    selectedLevel = "L4";
    updateLevelButtons();
  }

  private void onButtonPress(ActionEvent event) {
    if (!(event.getSource() instanceof Button)) {
      return;
    }
    Button button = (Button) event.getSource();
    selectedDirection = button.getText();

    resetSelections();
    updateCompassButtons();
    showSelectionBox(button);
    updatePendingCommand();
  }

  private void showSelectionBox(Button sourceButton) {
    double buttonX = sourceButton.getLayoutX();
    double buttonY = sourceButton.getLayoutY();

    selectionBox.setLayoutX(buttonX + sourceButton.getWidth() + 10);
    selectionBox.setLayoutY(buttonY);

    if (!selectionBox.isVisible()) {
      selectionBox.setVisible(true);
      FadeTransition ft = new FadeTransition(Duration.millis(200), selectionBox);
      ft.setFromValue(0.0);
      ft.setToValue(1.0);
      ft.play();
    }
  }

  private void updateCompassButtons() {
    for (Map.Entry<String, Button> entry : buttonMap.entrySet()) {
      Button button = entry.getValue();
      if (entry.getKey().equals(selectedDirection)) {
        button.setStyle(SELECTED_BUTTON_STYLE);
      } else {
        button.setStyle(DEFAULT_BUTTON_STYLE);
      }
    }
  }

  private void updateLevelButtons() {
    for (Map.Entry<String, Button> entry : levelButtonMap.entrySet()) {
      Button button = entry.getValue();
      if (entry.getKey().equals(selectedLevel)) {
        button.setStyle(SELECTED_LEVEL_STYLE);
      } else {
        button.setStyle(LEVEL_BUTTON_STYLE);
      }
    }
  }

  private void updatePendingCommand() {
    if (selectedDirection != null) {
      pendingCommand = selectedDirection + selectedSide + "_" + selectedLevel;
    }
  }

  private void sendToNetworkTables() {
    if (pendingCommand != null) {
      client.publish("moveTo", pendingCommand);
      System.out.println(pendingCommand);
      selectionBox.setVisible(false);

      System.out.println(pendingCommand);

      // Visual feedback
      goButton.setStyle(GO_BUTTON_PRESSED_STYLE);
      new Thread(
              () -> {
                try {
                  Thread.sleep(200);
                  javafx.application.Platform.runLater(
                      () -> {
                        goButton.setStyle(GO_BUTTON_STYLE);
                      });
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              })
          .start();
    }
  }

  private void refresh() {
    String moveTo = client.getValue("moveTo");
    System.out.println("Refresh Pressed: " + moveTo);

    if (moveTo != null && !moveTo.equals("not found")) {
      String[] parts = moveTo.split("_");
      if (parts.length == 3) {
        selectedDirection = parts[0];
        selectedSide = parts[1];
        selectedLevel = parts[2];
        pendingCommand = moveTo;

        updateCompassButtons();
        updateLevelButtons();
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

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
import javafx.scene.control.ToggleButton;
import javafx.scene.image.Image;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundImage;
import javafx.scene.layout.BackgroundPosition;
import javafx.scene.layout.BackgroundRepeat;
import javafx.scene.layout.BackgroundSize;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

public class ControllerApp extends Application {

  private static final String DEFAULT_BUTTON_STYLE = null;
  private static final String SELECTED_BUTTON_STYLE = "-fx-background-color: green";
  private static final String LEVEL_BUTTON_STYLE = "-fx-background-color: lightblue";
  private static final String SELECTED_LEVEL_STYLE = "-fx-background-color: blue";
  private static final String SIDE_BUTTON_STYLE = "-fx-background-color: lightgray";
  private static final String SELECTED_SIDE_STYLE = "-fx-background-color: green";

  private final NetworkTableClient client = NetworkTableClient.getInstance();
  private final Map<String, Button> compassButtonMap = new HashMap<>();
  private final Map<String, Button> levelButtonMap = new HashMap<>();
  private final Map<String, ToggleButton> sideButtonMap = new HashMap<>();
  private String pendingCommand = null;

  private String selectedDirection = null;
  private String selectedSide = null;
  private String selectedLevel = null;

  private static final String[] compassPoints = {"S", "SW", "NW", "N", "NE", "SE"};
  private static final String[] levels = {"L1", "L2", "L3", "L4"};
  private static final String[] sides = {"Left", "Right"};

  private Pane mainPane;
  private VBox topButtonBox;
  private VBox sideButtonBox;
  private HBox bottomButtonBox;
  private Pane compassButtonsPane;
  private VBox topSectionBox; // VBox to hold levels and sides

  @Override
  public void start(Stage primaryStage) {
    primaryStage.setTitle("Nemesis Controller");
    BorderPane root = new BorderPane();
    root.setPadding(new Insets(20));

    mainPane = new Pane();
    Image backgroundImage =
        new Image(getClass().getResource("/field-rotated-cropped.png").toExternalForm());

    BackgroundImage background =
        new BackgroundImage(
            backgroundImage,
            BackgroundRepeat.NO_REPEAT,
            BackgroundRepeat.NO_REPEAT,
            BackgroundPosition.CENTER,
            new BackgroundSize(100, 100, true, true, true, false));

    mainPane.setBackground(new Background(background));

    // Add listeners for responsive sizing
    mainPane.widthProperty().addListener((obs, oldVal, newVal) -> updateCompassButtonPositions());
    mainPane.heightProperty().addListener((obs, oldVal, newVal) -> updateCompassButtonPositions());

    compassButtonsPane = createCompassButtons();
    mainPane.getChildren().add(compassButtonsPane);

    topButtonBox = createLevelButtons();
    sideButtonBox = createSideButtons();
    bottomButtonBox = createBottomButtons();

    topSectionBox = new VBox(10); // Create VBox to hold levels and sides
    topSectionBox.setAlignment(Pos.TOP_CENTER);
    topSectionBox
        .getChildren()
        .addAll(topButtonBox, sideButtonBox); // Add level buttons then side buttons

    BorderPane.setAlignment(topSectionBox, Pos.TOP_CENTER); // Align top section to top center
    BorderPane.setAlignment(bottomButtonBox, Pos.BOTTOM_CENTER);
    BorderPane.setAlignment(mainPane, Pos.CENTER);

    root.setTop(topSectionBox); // Set the combined top section
    root.setCenter(mainPane);
    root.setBottom(bottomButtonBox);

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

  private Pane createCompassButtons() {
    Pane compassPane = new Pane();
    EventHandler<ActionEvent> buttonHandler = event -> onCompassButtonPress(event);

    for (String point : compassPoints) {
      Button button = new Button(point);
      compassButtonMap.put(point, button);
      button.setOnAction(buttonHandler);
      button.setPrefWidth(80);
      button.setPrefHeight(40);
      button.setStyle(DEFAULT_BUTTON_STYLE);
      compassPane.getChildren().add(button);
    }

    return compassPane;
  }

  private void updateCompassButtonPositions() {
    double width = mainPane.getWidth();
    double height = mainPane.getHeight();

    // Calculate center point
    double centerX = width / 2;
    // Moved center point to 2/3 down from the top (1/3 up from bottom)
    double centerY = height * (1.0 / 2.0);

    double radius = Math.min(width, height) * 0.125;

    double angleStep = Math.PI / 3;
    double startAngle = Math.PI / 2;

    for (int i = 0; i < compassPoints.length; i++) {
      Button button = compassButtonMap.get(compassPoints[i]);
      double angle = i * angleStep + startAngle;

      double x = centerX + radius * Math.cos(angle);
      double y = centerY + radius * Math.sin(angle);

      button.setLayoutX(x - button.getPrefWidth() / 2);
      button.setLayoutY(y - button.getPrefHeight() / 2);
    }
  }

  private VBox createLevelButtons() {
    VBox levelBox = new VBox(10);
    levelBox.setAlignment(Pos.TOP_CENTER);
    HBox buttonRow = new HBox(10);
    buttonRow.setAlignment(Pos.CENTER);

    for (String level : levels) {
      Button levelButton = new Button(level);
      levelButtonMap.put(level, levelButton);
      levelButton.setStyle(LEVEL_BUTTON_STYLE);
      levelButton.setPrefWidth(80);
      levelButton.setPrefHeight(40);

      levelButton.setOnAction(
          e -> {
            selectedLevel = level;
            updateLevelButtons();
            updatePendingCommand();
          });
      buttonRow.getChildren().add(levelButton);
    }
    levelBox.getChildren().add(buttonRow);
    return levelBox;
  }

  private VBox createSideButtons() {
    VBox sideBox = new VBox(20);
    sideBox.setAlignment(Pos.TOP_CENTER); // Center alignment for side buttons in the VBox
    sideBox.setPadding(
        new Insets(10, 0, 0, 0)); // Add some top padding to separate from level buttons

    HBox buttonRow = new HBox(10); // Use HBox to arrange Left and Right side by side
    buttonRow.setAlignment(Pos.CENTER); // Center buttons in HBox

    for (String side : sides) {
      ToggleButton sideButton = new ToggleButton(side);
      sideButtonMap.put(side, sideButton);
      sideButton.setStyle(SIDE_BUTTON_STYLE);
      sideButton.setPrefWidth(80);
      sideButton.setPrefHeight(40);

      sideButton.setOnAction(
          e -> {
            selectedSide = side;
            updateSideButtons();
            updatePendingCommand();
          });
      buttonRow.getChildren().add(sideButton);
    }
    sideBox.getChildren().add(buttonRow); // Add the HBox of side buttons to the VBox
    return sideBox;
  }

  private HBox createBottomButtons() {
    HBox buttonBox = new HBox(10);
    buttonBox.setAlignment(Pos.BOTTOM_CENTER);
    buttonBox.setPadding(new Insets(20));

    Button connectButton = new Button("Connect");
    connectButton.setOnAction(event -> connect());

    Button refreshButton = new Button("Refresh");
    refreshButton.setOnAction(event -> refresh());

    buttonBox.getChildren().addAll(connectButton, refreshButton);
    return buttonBox;
  }

  private void updateCompassButtons() {
    for (Map.Entry<String, Button> entry : compassButtonMap.entrySet()) {
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

  private void updateSideButtons() {
    for (Map.Entry<String, ToggleButton> entry : sideButtonMap.entrySet()) {
      ToggleButton button = entry.getValue();
      if (entry.getKey().equals(selectedSide)) {
        button.setStyle(SELECTED_SIDE_STYLE);
      } else {
        button.setStyle(SIDE_BUTTON_STYLE);
      }
    }
    if (selectedSide != null) {
      for (Map.Entry<String, ToggleButton> entry : sideButtonMap.entrySet()) {
        if (!entry.getKey().equals(selectedSide)) {
          entry.getValue().setSelected(false);
          entry.getValue().setStyle(SIDE_BUTTON_STYLE);
        } else {
          entry.getValue().setStyle(SELECTED_SIDE_STYLE);
          entry.getValue().setSelected(true);
        }
      }
    }
  }

  private void updatePendingCommand() {
    if (selectedLevel != null && selectedSide != null && selectedDirection != null) {
      pendingCommand = selectedDirection + "_" + selectedSide + "_" + selectedLevel;
      System.out.println("Updated command string: " + pendingCommand);
      sendToNetworkTables(pendingCommand);
    }
  }

  private void sendToNetworkTables(String command) {
    if (command != null) {
      client.publish("moveTo", command);
      System.out.println("Sending command: " + command);
    }
  }

  private void refresh() {
    String moveTo = client.getValue("moveTo");

    if (moveTo != null && !moveTo.equals("not found")) {
      String[] parts = moveTo.split("_");
      if (parts.length == 3) {
        selectedDirection = parts[0];
        selectedSide = parts[1];
        selectedLevel = parts[2];
        pendingCommand = moveTo;

        updateCompassButtons();
        updateLevelButtons();
        updateSideButtons();
      }
    }
  }

  private void connect() {
    client.connect();
  }

  private void onCompassButtonPress(ActionEvent event) {
    if (!(event.getSource() instanceof Button)) {
      return;
    }
    Button button = (Button) event.getSource();
    selectedDirection = button.getText();

    updateCompassButtons();
    updatePendingCommand();
  }

  public static void main(String[] args) {
    launch(args);
  }
}

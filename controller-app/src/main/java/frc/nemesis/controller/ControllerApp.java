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
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.stage.Stage;

public class ControllerApp extends Application {

  private static final String DEFAULT_BUTTON_STYLE =
      "-fx-background-color: #0080FF; -fx-text-fill: white; -fx-font-weight: bold";
  private static final String SELECTED_BUTTON_STYLE =
      "-fx-background-color: #00CC66; -fx-text-fill: white; -fx-font-weight: bold";

  private final NetworkTableClient client = NetworkTableClient.getInstance();
  private final Map<String, Button> levelButtonMap = new HashMap<>();
  private String pendingCommand = null;
  private String selectedDirection = null;

  // All button configurations
  private static final String[][] buttonConfigs = {
    {"Left-L4", "Right-L4"},
    {"Left-L3", "Right-L3"},
    {"Left-L2", "Right-L2"}
  };

  private GridPane mainButtonGrid;
  private HBox bottomButtonBox;

  @Override
  public void start(Stage primaryStage) {
    primaryStage.setTitle("Nemesis Controller");
    BorderPane root = new BorderPane();
    root.setPadding(new Insets(20));

    // Create main grid of buttons
    mainButtonGrid = createMainButtons();
    bottomButtonBox = createBottomButtons();

    BorderPane.setAlignment(mainButtonGrid, Pos.CENTER);
    BorderPane.setAlignment(bottomButtonBox, Pos.BOTTOM_CENTER);

    root.setCenter(mainButtonGrid);
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

  private GridPane createMainButtons() {
    GridPane grid = new GridPane();
    grid.setAlignment(Pos.CENTER);
    grid.setPadding(new Insets(20));
    grid.setHgap(20);
    grid.setVgap(20);

    EventHandler<ActionEvent> buttonHandler = event -> onButtonPress(event);

    // Create and add all buttons to grid
    for (int row = 0; row < buttonConfigs.length; row++) {
      for (int col = 0; col < buttonConfigs[row].length; col++) {
        String buttonText = buttonConfigs[row][col];
        Button button = new Button(buttonText);
        levelButtonMap.put(buttonText, button);
        button.setOnAction(buttonHandler);
        button.setStyle(DEFAULT_BUTTON_STYLE);

        // Make buttons expand to fill available space
        button.setMaxWidth(Double.MAX_VALUE);
        button.setMaxHeight(Double.MAX_VALUE);
        GridPane.setHgrow(button, Priority.ALWAYS);
        GridPane.setVgrow(button, Priority.ALWAYS);

        // The grid is built from top to bottom, but we want buttons from bottom to top
        grid.add(button, col, buttonConfigs.length - row - 1);
      }
    }

    return grid;
  }

  private HBox createBottomButtons() {
    HBox buttonBox = new HBox(10);
    buttonBox.setAlignment(Pos.BOTTOM_CENTER);
    buttonBox.setPadding(new Insets(20, 0, 0, 0));

    Button connectButton = new Button("Connect");
    connectButton.setOnAction(event -> connect());

    Button refreshButton = new Button("Refresh");
    refreshButton.setOnAction(event -> refresh());

    buttonBox.getChildren().addAll(connectButton, refreshButton);
    return buttonBox;
  }

  private void updateButtons() {
    for (Map.Entry<String, Button> entry : levelButtonMap.entrySet()) {
      Button button = entry.getValue();
      if (entry.getKey().equals(selectedDirection)) {
        button.setStyle(SELECTED_BUTTON_STYLE);
      } else {
        button.setStyle(DEFAULT_BUTTON_STYLE);
      }
    }
  }

  private void updatePendingCommand() {
    if (selectedDirection != null) {
      // Parse the button text to extract side and level
      String[] parts = selectedDirection.split("-");
      if (parts.length == 2) {
        String side = parts[0];
        String level = parts[1];

        // We need to send a command with format: Direction_Side_Level
        // For simplicity, we'll always use "N" as direction since it's not relevant anymore
        pendingCommand = "N_" + side + "_" + level;
        System.out.println("Updated command string: " + pendingCommand);
        sendToNetworkTables("moveTo", pendingCommand);
      }
    }
  }

  private void sendToNetworkTables(String key, String command) {
    if (command != null) {
      client.publish(key, command);
      System.out.println("Sending command: " + command);
    }
  }

  private void refresh() {
    String moveTo = client.getValue("moveTo");

    if (moveTo != null && !moveTo.equals("not found")) {
      // Split on underscore to separate parts
      String[] parts = moveTo.split("_");
      if (parts.length >= 3) {
        String side = parts[1];
        String level = parts[2];

        // Create button text format: "Side-Level"
        selectedDirection = side + "-" + level;
        updateButtons();
      }
    }
  }

  private void connect() {
    client.connect();
  }

  private void onButtonPress(ActionEvent event) {
    if (!(event.getSource() instanceof Button)) {
      return;
    }
    Button button = (Button) event.getSource();
    selectedDirection = button.getText();

    updateButtons();
    updatePendingCommand();
  }

  public static void main(String[] args) {
    launch(args);
  }
}

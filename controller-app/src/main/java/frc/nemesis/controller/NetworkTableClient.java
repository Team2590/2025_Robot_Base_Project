package frc.nemesis.controller;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import java.io.IOException;

/**
 * Client for communicating with the Robot and Driver Station through NetworkTables.
 *
 * <p>See https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html
 */
class NetworkTableClient {

  private static final String TABLE_NAME = "ControllerApp/target";

  private static final NetworkTableClient instance = new NetworkTableClient();

  private final NetworkTableInstance networkTable;

  private NetworkTableClient() {
    initialize();
    networkTable = NetworkTableInstance.getDefault();
    connect();
  }

  public static final NetworkTableClient getInstance() {
    return instance;
  }

  public void connect() {
    if (networkTable.isConnected()) {
      // Already connected
      // System.out.println(" -- Already Connected -- ");
      return;
    }
    networkTable.setServer("localhost");
    networkTable.startClient4("Nemesis Controller");
    // recommended if running on DS computer; this gets the robot IP from the DS
    networkTable.startDSClient();
  }

  public void disconnect() {
    networkTable.disconnect();
    networkTable.close();
  }

  public void publish(String key, String value) {
    NetworkTableEntry entry = getEntry(key);
    entry.setString(value);
    entry.close();
    networkTable.flush();
  }

  public String getValue(String key) {
    return getEntry(key).getString("not found");
  }

  private NetworkTableEntry getEntry(String key) {
    connect();
    NetworkTable table = networkTable.getTable(TABLE_NAME);
    return table.getEntry(key);
  }

  private static void initialize() {
    try {
      NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
      WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
      CombinedRuntimeLoader.loadLibraries(NetworkTableClient.class, "wpiutiljni", "ntcorejni");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}

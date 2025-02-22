package frc.robot;

// import frc.robot.Constants.BlueCoralPoses;
// import frc.robot.Constants.RedCoralPoses;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class ControllerOrchestrator extends SubsystemBase {
  private HashMap<String, Pose2d> poseMap = new HashMap<String, Pose2d>();
  private HashMap<String, Double> elevatorSetpointMap = new HashMap<String, Double>();
  private HashMap<String, Double> endeffectorWristSetpointMap = new HashMap<String, Double>();

  public ControllerOrchestrator() {
    // Optional<Alliance> ally = DriverStation.getAlliance();
    String alliance = "Red";
    switch (alliance) {
      case "Red":
        poseMap.put("Sright", FieldConstants.RedReefPoses.Sright);
        poseMap.put("Sleft", FieldConstants.RedReefPoses.Sleft);
        poseMap.put("SWright", FieldConstants.RedReefPoses.SWright);
        poseMap.put("SWleft", FieldConstants.RedReefPoses.SWleft);
        poseMap.put("NWright", FieldConstants.RedReefPoses.NWright);
        poseMap.put("NWleft", FieldConstants.RedReefPoses.NWleft);
        poseMap.put("Nright", FieldConstants.RedReefPoses.Nright);
        poseMap.put("Nleft", FieldConstants.RedReefPoses.Nleft);
        poseMap.put("NEright", FieldConstants.RedReefPoses.NEright);
        poseMap.put("NEleft", FieldConstants.RedReefPoses.NEleft);
        poseMap.put("SEright", FieldConstants.RedReefPoses.SEright);
        poseMap.put("SEleft", FieldConstants.RedReefPoses.SEleft);
        break;
      case "Blue":
        poseMap.put("Sright", FieldConstants.BlueReefPoses.Sright);
        poseMap.put("Sleft", FieldConstants.BlueReefPoses.Sleft);
        poseMap.put("SWright", FieldConstants.BlueReefPoses.SWright);
        poseMap.put("SWleft", FieldConstants.BlueReefPoses.SWleft);
        poseMap.put("NWright", FieldConstants.BlueReefPoses.NWright);
        poseMap.put("NWleft", FieldConstants.BlueReefPoses.NWleft);
        poseMap.put("Nright", FieldConstants.BlueReefPoses.Nright);
        poseMap.put("Nleft", FieldConstants.BlueReefPoses.Nleft);
        poseMap.put("NEright", FieldConstants.BlueReefPoses.NEright);
        poseMap.put("NEleft", FieldConstants.BlueReefPoses.NEleft);
        poseMap.put("SEright", FieldConstants.BlueReefPoses.SEright);
        poseMap.put("SEleft", FieldConstants.BlueReefPoses.SEleft);
        break;
    }

    // TODO: add real values
    elevatorSetpointMap.put("station", Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS);
    elevatorSetpointMap.put("L1", 10.0); // need to find
    elevatorSetpointMap.put("L2", Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS);
    elevatorSetpointMap.put("L3", Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS);
    elevatorSetpointMap.put("L4", Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS);

    // TODO: add real values
    endeffectorWristSetpointMap.put("station", 0.0);
    endeffectorWristSetpointMap.put("L1", 1.0);
    endeffectorWristSetpointMap.put("L2", 2.0);
    endeffectorWristSetpointMap.put("L3", 3.0);
    endeffectorWristSetpointMap.put("L4", 4.0);
  }


  public NetworkTableEntry getTableEntry(String key) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("ControllerApp/target");
    return table.getEntry(key);
  }

  public String getValue(String key) {
    return getTableEntry(key).getString("not found");
  }

  public void print() {
    System.out.println("This is " + getValue("moveTo"));
  }

  public Pose2d getTargetPose() {
    String poseKey = getValue("moveTo").split("_")[0];
    Pose2d pose = poseMap.get(poseKey);
    if (pose == null) {
      return new Pose2d();
    }
    return pose;
  }

  public double getElevatorSetpoint() {
    String elevatorSetpointKey = getValue("moveTo").split("_")[1];
    return elevatorSetpointMap.get(elevatorSetpointKey);
  }

  public double endeffectorWristSetpoint() {
    String endeffectorWristSetpointKey = getValue("moveTo").split("_")[0];
    return endeffectorWristSetpointMap.get(endeffectorWristSetpointKey);
  }
}

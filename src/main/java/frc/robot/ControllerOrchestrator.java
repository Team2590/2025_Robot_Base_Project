package frc.robot;

// import frc.robot.Constants.BlueCoralPoses;
// import frc.robot.Constants.RedCoralPoses;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ControllerOrchestrator extends SubsystemBase {
  private HashMap<String, Pose2d> poseMap = new HashMap<String, Pose2d>();
  private HashMap<String, Double> elevatorSetpointMap = new HashMap<String, Double>();
  private HashMap<String, Double> armSetpointMap = new HashMap<String, Double>();
  private String alliance;

  public ControllerOrchestrator() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        alliance = "Red";
      }
      if (ally.get() == Alliance.Blue) {
        alliance = "Blue";
      }
    } else {
      // SIM mode
      alliance = "Red";
    }
    switch (alliance) {
      case "Red":
        poseMap.put("Sright", FieldConstants.RedReefPoses.S_right);
        poseMap.put("Sleft", FieldConstants.RedReefPoses.S_left);
        poseMap.put("SWright", FieldConstants.RedReefPoses.SW_right);
        poseMap.put("SWleft", FieldConstants.RedReefPoses.SW_left);
        poseMap.put("NWright", FieldConstants.RedReefPoses.NW_right);
        poseMap.put("NWleft", FieldConstants.RedReefPoses.NW_left);
        poseMap.put("Nright", FieldConstants.RedReefPoses.N_right);
        poseMap.put("Nleft", FieldConstants.RedReefPoses.N_left);
        poseMap.put("NEright", FieldConstants.RedReefPoses.NE_right);
        poseMap.put("NEleft", FieldConstants.RedReefPoses.NE_left);
        poseMap.put("SEright", FieldConstants.RedReefPoses.SE_right);
        poseMap.put("SEleft", FieldConstants.RedReefPoses.SE_left);

        // Test
        poseMap.put("sourceTop", FieldConstants.CoralStationRight);
        poseMap.put("sourceBottom", FieldConstants.CoralStationLeft);
        break;

      case "Blue":
        poseMap.put("Sright", FieldConstants.BlueReefPoses.S_right);
        poseMap.put("Sleft", FieldConstants.BlueReefPoses.S_left);
        poseMap.put("SWright", FieldConstants.BlueReefPoses.SW_right);
        poseMap.put("SWleft", FieldConstants.BlueReefPoses.SW_left);
        poseMap.put("NWright", FieldConstants.BlueReefPoses.NW_right);
        poseMap.put("NWleft", FieldConstants.BlueReefPoses.NW_left);
        poseMap.put("Nright", FieldConstants.BlueReefPoses.N_right);
        poseMap.put("Nleft", FieldConstants.BlueReefPoses.N_left);
        poseMap.put("NEright", FieldConstants.BlueReefPoses.NE_right);
        poseMap.put("NEleft", FieldConstants.BlueReefPoses.NE_left);
        poseMap.put("SEright", FieldConstants.BlueReefPoses.SE_right);
        poseMap.put("SEleft", FieldConstants.BlueReefPoses.SE_left);
        break;
    }

    // TODO: add real values
    elevatorSetpointMap.put("source", Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS);
    elevatorSetpointMap.put(
        "L1", Constants.ElevatorConstantsLeonidas.ELEVATOR_SOURCE_POS); // need to find
    elevatorSetpointMap.put("L2", Constants.ElevatorConstantsLeonidas.ELEVATOR_L2_POS);
    elevatorSetpointMap.put("L3", Constants.ElevatorConstantsLeonidas.ELEVATOR_L3_POS);
    elevatorSetpointMap.put("L4", Constants.ElevatorConstantsLeonidas.ELEVATOR_L4_POS);

    // TODO: add real values
    armSetpointMap.put("source", Constants.ArmConstantsLeonidas.ARM_INTAKE_SOURCE_POSITION);
    armSetpointMap.put("L1", Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS);
    armSetpointMap.put("L2", Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS);
    armSetpointMap.put("L3", Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS);
    armSetpointMap.put("L4", Constants.ArmConstantsLeonidas.ARM_SCORING_CORAL_POS_L4);
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
    String elevatorSetpointKey;
    try {
      elevatorSetpointKey = getValue("moveTo").split("_")[1];
    } catch (Exception e) {
      elevatorSetpointKey = "source";
      System.out.println("E STACK " + e);
    }
    Logger.recordOutput("ControllerApp/ElevatorSetpoint", elevatorSetpointKey);
    return elevatorSetpointMap.get(elevatorSetpointKey);
  }

  public Command getElevatorSetpoint(Elevator elevator) {
    HashSet<Subsystem> requirements = new HashSet<>();
    requirements.add(elevator);
    return Commands.defer(
        () -> {
          Double elevatorSetpoint = getElevatorSetpoint();
          return elevator.setPosition(elevatorSetpoint);
        },
        requirements);
  }

  public double getArmSetpoint() {
    String armSetpointKey;
    try {
      armSetpointKey = getValue("moveTo").split("_")[1];
    } catch (Exception e) {
      armSetpointKey = "source";
    }
    Logger.recordOutput("ControllerApp/ArmSetpoint", armSetpointKey);
    return armSetpointMap.get(armSetpointKey);
  }
}

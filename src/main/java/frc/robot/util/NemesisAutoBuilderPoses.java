package frc.robot.util;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.util.NemesisAutoBuilder.ReefTarget;

public class NemesisAutoBuilderPoses {
  private static HashMap<String, Supplier<Pose2d>> map = new HashMap<>();

  static {
    map.put("S_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.S_left 
            : FieldConstants.BlueReefPoses.S_left);

    map.put("S_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.S_right 
            : FieldConstants.BlueReefPoses.S_right);

    map.put("SW_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.SW_left 
            : FieldConstants.BlueReefPoses.SW_left);

    map.put("SW_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.SW_right 
            : FieldConstants.BlueReefPoses.SW_right);

    map.put("NW_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.NW_right 
            : FieldConstants.BlueReefPoses.NW_right);

    map.put("NW_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.NW_left 
            : FieldConstants.BlueReefPoses.NW_left);

    map.put("N_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.N_right 
            : FieldConstants.BlueReefPoses.N_right);

    map.put("N_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.N_left 
            : FieldConstants.BlueReefPoses.N_left);

    map.put("NE_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.NE_right 
            : FieldConstants.BlueReefPoses.NE_right);

    map.put("NE_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.NE_left 
            : FieldConstants.BlueReefPoses.NE_left);

    map.put("SE_RIGHT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.SE_right 
            : FieldConstants.BlueReefPoses.SE_right);

    map.put("SE_LEFT", () -> 
        DriverStation.getAlliance().get() == Alliance.Red 
            ? FieldConstants.RedReefPoses.SE_left 
            : FieldConstants.BlueReefPoses.SE_left);
    }

  public static Pose2d getPose(ReefTarget reefTarget) {
    return map.get(reefTarget.name()).get();
  }
}

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.TreeMap;

public class ArmOptLookup {
  // Add optimal ArmPosition given an Elevator Position so we can synchronize movement optimally

  private double[] key_values;
  private double[] values;
  private TreeMap<Double, Double> acceptableArmPosition;

  public ArmOptLookup(double[] x, double[] y) {
    acceptableArmPosition = new TreeMap<>();
    key_values = x;
    values = y;

    for (int i = 0; i < x.length; i++) {
      acceptableArmPosition.put(x[i], y[i]);
    }
  }

  // linear interpolation
  public double get(double xval) {
    // double xval = RobotContainer.getElevator().getRotationCount();

    double dist = 100;
    int lower_i = 0;
    int upper_i = 0;
    for (int i = 0; i < key_values.length - 1; i++) {
      if (xval - key_values[i] < dist && xval - key_values[i] > 0) {
        dist = Math.abs(key_values[i] - xval);
        lower_i = i;
        upper_i = i + 1;
      }
    }

    double slope =
        (values[upper_i] - values[lower_i]) / (key_values[upper_i] - key_values[lower_i]);
    double deltaX = xval - key_values[lower_i];
    double deltaY = (xval < Constants.ArmConstantsLeonidas.OPT_TABLE_CLAMP) ? 0 : slope * deltaX;

    // Return value from the TreeMap if xval exists in it, otherwise perform interpolation
    return acceptableArmPosition.getOrDefault(xval, values[lower_i] + deltaY);
  }

  public Command populate(Trigger t) {
    return Commands.run(
        () -> {
          // Resize the arrays dynamically (if possible) or handle array resizing manually
          // This example assumes arrays are resized externally or statically defined with enough
          // capacity
          double[] newKeyValues = new double[key_values.length + 1];
          double[] newValues = new double[values.length + 1];

          System.arraycopy(key_values, 0, newKeyValues, 0, key_values.length);
          System.arraycopy(values, 0, newValues, 0, values.length);

          newKeyValues[key_values.length] = RobotContainer.getElevator().getRotationCount();
          newValues[values.length] = RobotContainer.getArm().getSetpoint();

          key_values = newKeyValues;
          values = newValues;

          acceptableArmPosition.put(key_values[key_values.length - 1], values[values.length - 1]);
        });
  }
}

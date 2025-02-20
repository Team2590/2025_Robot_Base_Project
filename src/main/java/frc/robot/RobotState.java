package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffector.EndEffector;

public class RobotState extends SubsystemBase{
    // get the zone
    // whether coral is in the end effector or intake (or if we have at all)
    // do we have algae
    // 

    String currentZone;
    private EndEffector endEffector = RobotContainer.getEndEffector();
    public void periodic() {
        
    }

    public boolean hasCoral() {
        return endEffector.hasCoral();
    }

    public String GetCurrentZone() {
        return currentZone;
    }

}

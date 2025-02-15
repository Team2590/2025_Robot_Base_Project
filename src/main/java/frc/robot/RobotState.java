package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {
    String currentZone;
    public RobotState() {
        // TODO: initalize instance variables and objects
    }

    @Override
    public void periodic() {
        // TODO: check various states
        currentZone = Constants.locator.getZoneOfField(Robot.getRobotContainerInstance().getDrive().getPose());
    }

    public boolean hasCoralIntake() {
        // TODO: add logic based on intake
        return false;
    }

    public boolean hasCoralEndEffector() {
        // TODO: add logic based on intake
        return false;
    }

    public boolean hasCoralAlgae() {
        // TODO: add logic based on intake
        return false;
    }

    public String getCurrentZone() {
        return currentZone;
    }

}

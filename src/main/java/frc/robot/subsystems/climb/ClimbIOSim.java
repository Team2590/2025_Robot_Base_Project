package frc.robot.subsystems.climb;

public class ClimbIOSim implements ClimbIO {

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRads = 0.0;
    inputs.velocityRadsPerSec = 0.0;
    inputs.appliedVoltage = 0.0;
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = 0.0;
    inputs.tempCelsius = 0.0;
    inputs.rotationCount = 0.0;
    inputs.statorCurrentAmps = 0.0;
  }

  @Override
  public void setVoltage(double voltage) {
    // Do nothing in simulation
  }

  @Override
  public void setVelocity(double speed) {
    // Do nothing in simulation
  }

  @Override
  public void stop() {
    // Do nothing in simulation
  }

  @Override
  public void resetRotationCount() {
    // Do nothing in simulation
  }
}

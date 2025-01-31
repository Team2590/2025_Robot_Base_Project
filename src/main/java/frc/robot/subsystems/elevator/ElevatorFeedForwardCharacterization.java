package frc.robot.subsystems.elevator;

import frc.robot.commands.FeedForwardCharacterization;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ElevatorFeedForwardCharacterization extends FeedForwardCharacterization {
  Elevator elevator;
  private double minRotCounts;
  private double maxRotCounts;

  public ElevatorFeedForwardCharacterization(
      Elevator elevator,
      Consumer<Double> voltageConsumer,
      Supplier<Double> velocitySupplier,
      double minRotCounts,
      double maxRotCounts) {
    super(elevator, voltageConsumer, velocitySupplier);
    this.elevator = elevator;
    this.minRotCounts = minRotCounts;
    this.maxRotCounts = maxRotCounts;
  }

  @Override
  public boolean isFinished() {
    return elevator.getRotationCount() > maxRotCounts || elevator.getRotationCount() < minRotCounts;
  }
}

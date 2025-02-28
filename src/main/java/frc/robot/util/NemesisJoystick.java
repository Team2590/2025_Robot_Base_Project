package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashSet;

public class NemesisJoystick extends CommandJoystick {
  private HashSet<Integer> buttonIDs = new HashSet<Integer>();
  private static HashSet<Integer> ports = new HashSet<Integer>();

  public NemesisJoystick(int port) {
    super(port);
    if (ports.contains(port))
      throw new IllegalArgumentException("Port " + port + " already in use");
    ports.add(port);
  }

  @Override
  public Trigger button(int id) {
    if (buttonIDs.contains(id))
      throw new IllegalArgumentException("Button " + id + " already binded to an event");
    buttonIDs.add(id);
    return super.button(id);
  }
}

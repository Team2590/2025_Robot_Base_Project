package frc.robot.command_factories.LEDS;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NemesisLED extends SubsystemBase {
      // ----------CONSTANT(S)--------------\\
      private final int BUFFER_LENGTH = 304;
      // --------VARIABLE(S)--------\\
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object


      // ---------CONSTRUCTOR(S)----------------\
      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
      public LEDS(int port) {
            // TODO maybe pass in buffer length
            leds = new AddressableLED(port); // initialization of the AdressableLED
            leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once
            buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
            // TODO we start a starting color
            setBuffer(buffer);

            //TODO:SEE IF WE NEED THIS LINE
            leds.start();
      }

      /**
       * <h4>getBufferLength</h4>
       * Returns the buffer length
       * 
       * @return BUFFER_LENGTH
       */
      public int getBufferLength() {
            return BUFFER_LENGTH;
      } // End of getBufferLength()

      public AddressableLEDBuffer getBuffer() {
            return buffer;
      }
      public void setBuffer(AddressableLEDBuffer buffer) {
            leds.setData(buffer);
      }

}

/** 
 * ~~~~~MY CODE~~~~~
 * 
 * import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
    private static final AddressableLED m_led;
    private static final AddressableLEDBuffer m_buffer
    private final int length;
    private int hue = 0;
}

public LEDS {

    m_led = new AddressableLED(0);

}
 */


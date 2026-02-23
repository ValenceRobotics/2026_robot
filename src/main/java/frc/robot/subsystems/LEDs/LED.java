package frc.robot.subsystems.LEDs;

// These imports are necessary in order to publish everything to AdvantageScope
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FullSubsystem;

public class LED extends FullSubsystem {
  private final int TOTAl_TIME = 160;

  private final double SHIFT_1_START = 130.0;
  private final double SHIFT_1_END = 105.0;

  private final double SHIFT_2_START = 105.0;
  private final double SHIFT_2_END = 80.0;

  private final double SHIFT_3_START = 80.0;
  private final double SHIFT_3_END = 55.0;

  private final double SHIFT_4_START = 55.0;
  private final double SHIFT_4_END = 30.0;

  private double currentTime = TOTAl_TIME;
  private double recentTime;
  private boolean wonAuto = false;

  private final AddressableLED led;
  private final AddressableLEDBuffer led_buffer;

  public LED() {
    recentTime = Timer.getFPGATimestamp(); // record the start time
    led = new AddressableLED(0);
    led_buffer = new AddressableLEDBuffer(60);
    led.setLength(led_buffer.getLength());
    led.setData(led_buffer);
    led.start();
  }

  @Override
  public void periodicAfterScheduler() {
    double now = Timer.getFPGATimestamp(); // get current timestamp
    boolean canScore = false;

    if (currentTime > 0) {
      double delta = now - recentTime; // calculate time passed
      currentTime -= delta;
      boolean blinkToggle = (int) (currentTime * 5.0) % 2 == 0;

      if (wonAuto == true
          && ((currentTime <= SHIFT_2_START && currentTime >= SHIFT_2_END)
              || (currentTime <= SHIFT_4_START && currentTime >= SHIFT_4_END))) {
        canScore = true;
      }

      if (wonAuto == false
          && ((currentTime <= SHIFT_1_START && currentTime >= SHIFT_1_END)
              || (currentTime <= SHIFT_3_START && currentTime >= SHIFT_3_END))) {
        canScore = true;
      }

      for (int i = 0; i < led_buffer.getLength(); i++) {
        led_buffer.setRGB(i, 0, 0, 0);

        if (currentTime <= 35.0 && currentTime >= SHIFT_4_END) {
          if (blinkToggle) {
            led_buffer.setRGB(i, 255, 165, 0);
          }
        } else if (wonAuto == true) {
          if ((currentTime <= 110.0 && currentTime >= SHIFT_2_START)
              || (currentTime <= 60.0 && currentTime >= SHIFT_4_START)) {
            if (blinkToggle) {
              led_buffer.setRGB(i, 255, 165, 0);
            }
          } else if ((currentTime <= 135.0 && currentTime >= SHIFT_1_START)
              || (currentTime <= 85.0 && currentTime >= SHIFT_3_START)) {
            if (blinkToggle) {
              led_buffer.setRGB(i, 255, 255, 255);
            }
          }
        } else if (wonAuto == false) {
          if ((currentTime <= 135.0 && currentTime >= SHIFT_1_START)
              || (currentTime <= 85.0 && currentTime >= SHIFT_3_START)) {
            if (blinkToggle) {
              led_buffer.setRGB(i, 255, 165, 0);
            }
          } else if ((currentTime <= 110.0 && currentTime >= SHIFT_2_START)
              || (currentTime <= 60.0 && currentTime >= SHIFT_4_START)) {
            if (blinkToggle) {
              led_buffer.setRGB(i, 255, 255, 255);
            }
          }
        }
      }

      led.setData(led_buffer);

      NetworkTableInstance.getDefault()
          .getEntry("SmartDashboard/ALIVE_TIMER")
          .setDouble(currentTime); // publish to AdvantageScope
    }
    recentTime = now;
  }
}

package frc.robot.subsystems.candle;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;

/** IO interface for the Extender (one motor, position controlled). */
public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    public boolean lightConnected = false;
    public int ledsPerAnimation = 0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Helper class to make it easier to pass colours as an object. it limits the range of values to those of an RBG value. */
  public static class Colour {

    public int r;
    public int g;
    public int b;

    public Colour(int r, int g, int b) {
      this.r = clamp(r);
      this.g = clamp(g);
      this.b = clamp(b);
    }

    private int clamp(int value) {
      return Math.max(0, Math.min(255, value));
    }
  }

  /** Enum containing a list of common colours for easy reference */
  public enum CommonColours {

    RED(new Colour(255, 0, 0)),
    GREEN(new Colour(0, 255, 0)),
    BLUE(new Colour(0, 0, 255)),
    YELLOW(new Colour(255, 255, 0)),
    PURPLE(new Colour(128, 0, 128)),
    CYAN(new Colour(0, 255, 255)),
    WHITE(new Colour(255, 255, 255)),
    OFF(new Colour(0, 0, 0));

    public final Colour colour;

    CommonColours(Colour colour) {
      this.colour = colour;
    }
  }

  /** Update inputs from the hardware. */
  default void updateInputs(CANdleIOInputs inputs) { }

  /** Set the LEDs color. */
  default void setColor(Colour colour) { }

  /** Set the strobe animation of the LEDs */
  default void setStrobeAnimation(StrobeAnimation animation) { }

  /** Set the rainbow animation of the LEDs */
  default void setRainbowAnimation(RainbowAnimation animation) { }

  /** Clear the LEDs */
  default void clear() { }
}

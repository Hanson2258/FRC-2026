package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/** Field layout constants including target positions for different alliances. */
public final class FieldConstants {
  /** Blue alliance hub center position in meters. */
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6256194, 4.0346376);

  /** Red alliance hub center position in meters. */
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9154194, 4.0346376);

  private FieldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

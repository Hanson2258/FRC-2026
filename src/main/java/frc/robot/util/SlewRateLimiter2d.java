package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Slew-rate limits a 2D translation so acceleration is bounded. Uses magnitude-based limiting
 * (same rate limit for the vector magnitude).
 */
public class SlewRateLimiter2d {
  private final double rateLimit;
  private Translation2d prevTranslation = new Translation2d();
  private double prevTime = 0.0;

  /**
   * @param rateLimit Maximum rate of change of the vector magnitude (same unit as translation per
   *     second).
   */
  public SlewRateLimiter2d(double rateLimit) {
    this.rateLimit = rateLimit;
  }

  /**
   * Applies slew-rate limiting to the desired translation.
   *
   * @param translation Desired translation (e.g. velocity in m/s).
   * @return Limited translation.
   */
  public Translation2d calculate(Translation2d translation) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - prevTime;
    prevTime = currentTime;
    prevTranslation = MathUtil.slewRateLimit(prevTranslation, translation, dt, rateLimit);
    return prevTranslation;
  }

  /** Resets the limiter to the given translation (e.g. on command init). */
  public void reset(Translation2d translation) {
    prevTranslation = translation;
    prevTime = Timer.getFPGATimestamp();
  }
}

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the turret subsystem. */
public final class TurretConstants {

  private TurretConstants() {}

  /** CAN ID of the turret motor. */
  public static final int kMotorId = 5;

  /** Turret radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 20.0;

  /**
   * Encoder zero offset (radians). Added to raw encoder so that 0 = turret pointing robot-forward.
   * If when the turret is forward the encoder reads 0.5 rad, set this to -0.5.
   */
  public static final double kEncoderZeroOffsetRad = 0.0;

  /** PID gains for onboard position control and for sim software control. */
  public static final double kP = 7.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Minimum turret angle (radians). */
  public static final double kMinAngleRad = Units.degreesToRadians(-180.0);

  /** Maximum turret angle (radians). */
  public static final double kMaxAngleRad = Units.degreesToRadians(180.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Vision camera index used for hub aiming (when using fixed camera + tx). */
  public static final int kVisionCameraIndex = 0;

  /** Angle from robot forward to camera boresight (for fixed camera). */
  public static final Rotation2d kCameraAngleOffset = Rotation2d.kZero;
}

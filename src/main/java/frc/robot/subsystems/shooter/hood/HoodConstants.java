package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

/** Constants for the hood subsystem. */
public final class HoodConstants { // TODO: Add correct Values

  private HoodConstants() {}

  /** CAN ID of the hood motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 7; // TODO: Add correct ID

  /** Motor shaft rotations per hood rotation (output / input). */
  public static final double kGearRatio = 1.0;

  /**
   * Encoder zero offset (radians). Added to raw encoder so that 0 = defined physical position.
   */
  public static final double kEncoderZeroOffsetRad = 0.0;

  /** PID gains for onboard position control and for sim software control. */
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Minimum hood angle (radians). */
  public static final double kMinAngleRad = Units.degreesToRadians(0.0);

  /** Maximum hood angle (radians). */
  public static final double kMaxAngleRad = Units.degreesToRadians(60.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance (radians) for at-target check. */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);
}

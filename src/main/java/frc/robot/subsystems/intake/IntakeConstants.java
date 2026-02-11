package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

/** Constants for the intake subsystem. */
public final class IntakeConstants { // TODO: Add correct values

  private IntakeConstants() {}

  /** CAN ID of the intake motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 0; // TODO: Set correct CAN ID

  /** Output rotations per motor rotation (1.0 = 1:1). */
  public static final double kGearRatio = 1.0; // TODO: Set if not 1:1

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Set true if positive velocity spins the intake the opposite direction. */ // TODO: Add correct value
  public static final boolean kMotorInverted = false;

  /** Target velocity (rad/s, output shaft) when running. TODO: Tune for reliable intake. */
  public static final double kTargetVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);

  /** Velocity PIDF gains (onboard and sim). */
  public static final double kP = 0.0001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0002;
  public static final double kS = 0.0;
}

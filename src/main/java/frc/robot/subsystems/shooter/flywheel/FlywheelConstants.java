package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;

/** Constants for the flywheel subsystem. */
public final class FlywheelConstants {

  private FlywheelConstants() {}

  /** CAN ID of the flywheel motor. */
  public static final int kMotorId = 6; // TODO: Add correct ID

  /** Flywheel radius (m) for converting angular velocity to launch linear velocity. ~2 in. */
  public static final double kFlywheelRadiusMeters = 0.0508;

  /** Flywheel radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 1.0;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Stator current limit (amps); protects motor and gearbox. */
  public static final double kStatorCurrentLimitAmps = 40.0; // TODO: Tune for your motor

  /** Set true if positive velocity spins the flywheel the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Idle target velocity (rad/s). */
  public static final double kIdleVelocityRadsPerSec = 0.0;

  /** Target velocity (rad/s) for shooting. */
  public static final double kDefaultTargetVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000.0); // TODO: Add correct value

  /** Tolerance for at-target velocity (Charging → AtSpeed). */
  public static final double kAtTargetVelocityToleranceRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

  /** Velocity PIDF gains (onboard and sim). */
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.0;
  public static final double kS = 0.0;
}

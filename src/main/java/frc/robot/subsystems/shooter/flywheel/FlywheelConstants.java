package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;

/** Constants for the Flywheel subsystem. */
public final class FlywheelConstants { // XXX: Add correct values

  private FlywheelConstants() {}

  /** CAN ID of the Flywheel motor. */
  public static final int kMotorId = 7;

  /** Flywheel radius for converting angular velocity to launch linear velocity. ~2 in. */
  public static final double kFlywheelRadiusMeters = 0.0508;

  /** Flywheel radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 1.0;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Stator current limit (amps); protects motor and gearbox. */
  public static final double kStatorCurrentLimitAmps = 40.0;

  /** Max velocity ramp rate per second. */
  public static final double kVelocityRampRateRpmPerSec = 1000.0;

  /** Set true if positive velocity spins the Flywheel the opposite direction. */
  public static final boolean kMotorInverted = true;

  /** Target velocity in Idle state. */
  public static final double kIdleVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0.0);

  /** Target velocity for shooting. */
  public static final double kDefaultTargetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3200.0);

  /** Tolerance for at-target velocity (Charging → At_Speed). */
  public static final double kAtTargetVelocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

  /** Delta Rad per step. */
  public static final double kStepRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

  /** Velocity PIDF gains (onboard and sim). */
  public static final double kP = 0.1;
  public static final double kI = 0.00000;
  public static final double kD = 0.0;
  public static final double kV = 0.115;
  public static final double kS = 0.0;
}

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;

/** Constants for the flywheel subsystem. */
public final class FlywheelConstants {

  private FlywheelConstants() {}

  /** CAN ID of the flywheel motor. */
  public static final int kMotorId = 6; // TODO: Add correct ID

  /** Flywheel radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 1.0;

  /** Velocity PID gains for onboard control and for sim software control. */ // TODO: Add correct values
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Default target velocity (rad/s) for shooting; tune to ~50% equivalent. */
  public static final double kDefaultTargetVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000.0); // TODO: Add correct value

  /** Tolerance for the flywheel to be at target speed. */
  public static final double kAtSpeedToleranceRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(50.0);
}

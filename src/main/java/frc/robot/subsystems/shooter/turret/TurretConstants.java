package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the Turret (position-controlled hub aiming) subsystem. */
public final class TurretConstants { // XXX: Add correct values

  private TurretConstants() {}

  /** CAN ID of the Turret motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 10;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Neutral mode when output is zero (coast or brake). Talon FX only. */
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;

  /** Turret radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 42.0;

  /** PID gains for onboard position control and for sim software control. */
  public static final double kP = 7.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Encoder zero offset. Added to raw encoder so that 0 = Turret pointing robot-forward. */
  public static final double kEncoderZeroOffsetRad = 0;

  /** Minimum Turret angle. */
  public static final double kMinAngleRad = Units.degreesToRadians(-90.0);

  /** Maximum Turret angle. */
  public static final double kMaxAngleRad = Units.degreesToRadians(90.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Vision camera index used for hub aiming (when using fixed camera + tx). */
  public static final int kVisionCameraIndex = 0;

  /** Angle from robot forward to camera boresight (for fixed camera). */
  public static final Rotation2d kCameraAngleOffset = Rotation2d.kZero;
}

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/** Constants for the Hood (position-controlled shooter angle) subsystem. */
public final class HoodConstants { // TODO: Add correct values

  private HoodConstants() {}

  /** CAN ID of the Hood motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 7;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Neutral mode when output is zero (coast or brake). Talon FX only. */
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;

  /** Motor shaft rotations per Hood rotation (output / input). */
  public static final double kGearRatio = 1.0;

  /** PID gains for onboard position control and for sim software control. */
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Encoder zero offset. Added to raw encoder so that 0 = defined physical position. */
  public static final double kEncoderZeroOffsetRad = 0.0;

  /** Minimum Hood angle. */
  public static final double kMinAngleRad = Units.degreesToRadians(20.0);

  /** Maximum Hood angle. */
  public static final double kMaxAngleRad = Units.degreesToRadians(10.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance for at-target check. */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);
}

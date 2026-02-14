package frc.robot.subsystems.agitator;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the agitator (intake-to-transfer) subsystem. */
public final class AgitatorConstants { // TODO: Add correct values

  private AgitatorConstants() {}

  /** CAN ID of the agitator motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 20;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Set true if positive voltage spins the agitator the opposite direction. */
  public static final boolean kMotorInverted = true;

  /** Smart current limit (amps). */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Open-loop ramp time (s) from 0 to full output. Limits current spikes on step changes. */
  public static final double kOpenLoopRampRateSec = 0.3;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 8.0;

  /** Voltage when idle. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage when staging (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage when shooting (fast loading). */
  public static final double kShootingVoltage = 6.0;
}

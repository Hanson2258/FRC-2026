package frc.robot.subsystems.shooter.transfer;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the Transfer (agitator-to-shooter) subsystem. */
public final class TransferConstants { // TODO: Add correct values

  private TransferConstants() {}

  /** CAN ID of the Transfer motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 7;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Set true if positive voltage spins the Transfer the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Open-loop ramp time from 0 to full output. Limits current spikes on step changes. SPARK MAX only. */
  public static final double kOpenLoopRampRateSec = 0.3;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 10.0;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;

  /** Voltage when idle. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage when staging (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage when shooting. */
  public static final double kShootingVoltage = 6.0;

  /** Proximity at or above this value = ball present (REV Color Sensor V3: 0–2047). */
  public static final int kColorSensorProximityThreshold = 150;
}

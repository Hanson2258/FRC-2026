package frc.robot.subsystems.hang;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the Hang (climber) subsystem: control to pot voltage setpoints. */
public final class HangConstants { // XXX: Set correct values for your robot

  private HangConstants() {}

  /** CAN ID of the Hang motor (SPARK MAX). */
  public static final int kMotorId = 9;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kBrake;

  /** Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 40;

  /** Open-loop ramp rate (sec from 0 to full). */
  public static final double kOpenLoopRampRateSec = 0.2;

  /** Whether the motor output should be inverted. */
  public static final boolean kMotorInverted = true;

  /** Analog input channel for the hang potentiometer (RoboRIO2 port). */
  public static final int kPotChannel = 0;

  /**
   * Potentiometer voltage range: 5 V = retracted (stored), 5 V * 0.33 = extended (level 1).
   * Voltage decreases as mechanism extends. Control targets these voltages directly (no conversion).
   */ 
  public static final double kPotRetractedVoltage = 4.14; //  Was 4.41
  public static final double kPotExtendedVoltage = -1.0; // Was 1.4

  /** Preset target voltages (V): stored = retracted, level 1 = extended. */
  public static final double kStoredVoltage = kPotRetractedVoltage;
  public static final double kLevel1Voltage = kPotExtendedVoltage;

  /** PID gains for voltage setpoint control. */
  public static final double kP = 40.0;
  public static final double kI = 0.0;
  public static final double kD = 0.2;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 5.0;

  /** Potentiometer voltage tolerance for at-target checks (V). */
  public static final double kAtTargetToleranceVolts = 0.05;
}


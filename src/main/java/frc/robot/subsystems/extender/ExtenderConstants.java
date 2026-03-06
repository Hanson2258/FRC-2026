package frc.robot.subsystems.extender;

import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.util.Units;

/** Constants for the Extender (one motor, voltage controlled) subsystem. */
public class ExtenderConstants { // XXX: Add correct values

  /** CAN ID of the Extender motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 6;

  /** Idle behaviour when ouput is 0 (Coast or Brake)  */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;
  
  /** Set true to invert the motor */
  public static final boolean kMotorInverted = false;

  /** Smart current limit */
  public static int kSmartCurrentLimitAmps = 25;

  /** Extender radians per motor rotation 1.0 = 1:1 */
  public static final double kGearRatio = 48.0 * 18.0 / 38.0; // Gearbox is 48:1, small sprocket (on motor) has 18 teeth, big sprocket has 38 teeth.

  /** Target position when the extender is in the UP mode */
  public static final double kUpExtenderRads = Units.degreesToRadians(0);

  /** Target position when the extender is in the PARTIAL mode */
  public static final double kPartialExtenderRads = Units.degreesToRadians(60);

  /** Target position when the extender is in the DOWN mode */
  public static final double kDownExtenderRads = Units.degreesToRadians(90);

  /** Min radians for the extender to rotate */
  public static final double kMinRads = 0;

  /** Max radians for the extender to rotate */
  public static final double kMaxRads = Units.degreesToRadians(115);

  /** Tolerance for at-target position */
  public static final double kAtTargetRadsTolerance = Units.degreesToRadians(2.0);
  
  /** PID values for to-position target */
  public static final double kP = 0.1;
  public static final double kI = 0;
  public static final double kD = 0;
}

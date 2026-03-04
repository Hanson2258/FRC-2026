package frc.robot.subsystems.extender;

import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.util.Units;

public class ExtenderConstants {

  // TODO: get motor id
  /** CAN ID of the extender */
  public static final int kMotorId = 0;

  /** Idle behaviour when ouput is 0 (Coast or Brake)  */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;
  
  // TODO: tune
  /** Smart current limit */
  public static int kSmartCurrentLimitAmps = 25;

  // TODO: set gear ratio
  /** Extender radians per motor rotation 1.0 = 1:1 */
  public static final double kGearRatio = 1.0;

  // TODO: tune position
  /** Target position when the extender is in the UP mode */
  public static final double kUpExtenderRads = Units.degreesToRadians(90);

  // TODO: tune position
  /** Target position when the extender is in the DOWN mode */
  public static final double kDownExtenderRads = 0;

  /** Set true to invert the motor */
  public static final boolean kMotorInverted = false;

  // TODO: tune max pos
  /** Max radians for the extender to rotate */
  public static final double kMaxRads = Units.degreesToRadians(100);
  
  // TODO: tune min voltage
  /** Min radians for the extender to rotate */
  public static final double kMinRads = 0;

  // TODO: tune PID
  /** PID values for to-position target */
  public static final double kP = 0.1;
  public static final double kI = 0;
  public static final double kD = 0;

  // TODO: tune
  /** Tolerance for at-target position */
  public static final double kAtTargetRadsTolerance = 0.2;
}

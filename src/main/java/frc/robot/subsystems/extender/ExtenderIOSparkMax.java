package frc.robot.subsystems.extender;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import static frc.robot.subsystems.extender.ExtenderConstants.kD;
import static frc.robot.subsystems.extender.ExtenderConstants.kGearRatio;
import static frc.robot.subsystems.extender.ExtenderConstants.kI;
import static frc.robot.subsystems.extender.ExtenderConstants.kIdleMode;
import static frc.robot.subsystems.extender.ExtenderConstants.kMotorId;
import static frc.robot.subsystems.extender.ExtenderConstants.kP;
import static frc.robot.subsystems.extender.ExtenderConstants.kSmartCurrentLimitAmps;

public class ExtenderIOSparkMax implements ExtenderIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;
  private double targetPosition;
  
  public ExtenderIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
      .encoder
      .positionConversionFactor(1.0 / kGearRatio)
      .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);

    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End ExtenderIOSParkMax

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {

  } // End updateInputs

  @Override
  public void setTargetPosition(double rads) {
     double targetRot = Units.radiansToRotations(rads);
     closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition);
     targetPosition = rads;
  } // End setTargetPosition

  @Override
  public void resetEncoders() {
    encoder.setPosition(0.0);
  } // End resetEncoders

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

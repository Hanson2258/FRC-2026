package frc.robot.subsystems.extender;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.subsystems.extender.ExtenderConstants.*;

public class ExtenderIOSparkMax implements ExtenderIO {

  private static final int kSignalsPeriodMs = 31;
  private static final int kEncoderVelocitySignalPeriodMs = 31;

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
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
      .encoder
      .positionConversionFactor(1.0 / kGearRatio)
      .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD).maxOutput(0.3).minOutput(-0.3); // XXX: Tune to ensure enough power
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);

    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor.setPeriodicFrameTimeout(0);
  } // End ExtenderIOSparkMax Constructor

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    double p = SmartDashboard.getNumber("Extender/kP", kP);
    double i = SmartDashboard.getNumber("Extender/kI", kI);
    double d = SmartDashboard.getNumber("Extender/kD", kD);
    if (p != lastP || i != lastI || d != lastD) {

      lastP = p;
      lastI = i;
      lastD = d;

      SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
      sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
      sparkMaxConfig.signals
          .appliedOutputPeriodMs(kSignalsPeriodMs)
          .busVoltagePeriodMs(kSignalsPeriodMs)
          .outputCurrentPeriodMs(kSignalsPeriodMs)
          .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
          .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
      motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      motor.setPeriodicFrameTimeout(0);
    }

    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.targetPositionRads = targetPosition;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double positionRad) {
    double targetRot = Units.radiansToRotations(positionRad);
    closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition);
    targetPosition = positionRad;
  } // End setTargetPosition

  @Override
  public void resetEncoders() {
    encoder.setPosition(Units.radiansToRotations(kMaxRad));
  } // End resetEncoders

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

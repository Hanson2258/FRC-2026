package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/** Hood IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class HoodIOSparkMax implements HoodIO {

  private static final int kSignalsPeriodMs = 19;
  private static final int kEncoderVelocitySignalPeriodMs = 19;

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  public HoodIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor.setPeriodicFrameTimeout(0);
  } // End HoodIOSparkMax Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    double targetRot = Units.radiansToRotations(targetRads);
    closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition);
  } // End setTargetPosition

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

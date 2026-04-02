package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.robot.Constants;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

/** Intake IO using a single SPARK MAX (NEO 550) with voltage control. */
public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax motor;

  public IntakeIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig.openLoopRampRate(kOpenLoopRampRateSec);
    sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kEncoderVelocitySignalPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motor.setPeriodicFrameTimeout(0);
  } // End IntakeIOSparkMax Constructor

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setVoltage(double volts, boolean ignoreLimits) {
    double clamped = ignoreLimits
        ? MathUtil.clamp(volts, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    motor.setVoltage(clamped);
  } // End setVoltage

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

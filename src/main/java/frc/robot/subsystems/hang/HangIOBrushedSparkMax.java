package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

/** Hang IO using a single SPARK MAX (brushed) with an external analog potentiometer. */
public class HangIOBrushedSparkMax implements HangIO {

  private static final int kSignalsPeriodMs = 31;
  private static final int kEncoderVelocitySignalPeriodMs = 31;

  private final SparkMax motor;
  private final AnalogInput potentiometer;

  public HangIOBrushedSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushed);
    potentiometer = new AnalogInput(kPotChannel);
    potentiometer.setAverageBits(4);

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
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    motor.setPeriodicFrameTimeout(0);
  } // End HangIOBrushedSparkMax Constructor

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.potVoltage = potentiometer.getAverageVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    motor.setVoltage(clamped);
  }

  @Override
  public void stop() {
    motor.set(0.0);
  }
}

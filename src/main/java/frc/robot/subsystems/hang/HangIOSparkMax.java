package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

/** Hang IO using a single SPARK MAX with an external analog potentiometer. */
public class HangIOSparkMax implements HangIO {

  private final SparkMax motor;
  private final AnalogInput potentiometer;

  public HangIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    potentiometer = new AnalogInput(kPotChannel);
    // Average 16 samples for stable potentiometer readings (WPILib recommendation)
    potentiometer.setAverageBits(4);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig.openLoopRampRate(kOpenLoopRampRateSec);
    sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(37)
        .busVoltagePeriodMs(37)
        .outputCurrentPeriodMs(37)
        .primaryEncoderPositionPeriodMs(32719)
        .primaryEncoderVelocityPeriodMs(32719);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End HangIOSparkMax Constructor

  @Override
  public void updateInputs(HangIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.potVoltage = potentiometer.getAverageVoltage();
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  } // End setVoltage

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}


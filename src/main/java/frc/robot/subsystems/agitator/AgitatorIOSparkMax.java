package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import frc.robot.Constants;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

/** Agitator IO using a single SPARK MAX (NEO 550) with voltage control. */
public class AgitatorIOSparkMax implements AgitatorIO {

  private final SparkMax motor;

  public AgitatorIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig.openLoopRampRate(kOpenLoopRampRateSec);
    sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End AgitatorIOSparkMax Constructor

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setVoltage(double volts) {
    double clamped = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
    motor.setVoltage(clamped);
  } // End setVoltage

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

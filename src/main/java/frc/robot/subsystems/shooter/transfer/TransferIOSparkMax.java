package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

// import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.I2C;

/** Transfer IO using a single SPARK MAX (NEO 550) with voltage control and optional colour sensor. */
public class TransferIOSparkMax implements TransferIO {

  private final SparkMax motor;
  // private final ColorSensorV3 colorSensor;

  public TransferIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    // colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig.openLoopRampRate(kOpenLoopRampRateSec);
    sparkMaxConfig.voltageCompensation(Constants.kNominalVoltage);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(31)
        .busVoltagePeriodMs(31)
        .outputCurrentPeriodMs(31)
        .primaryEncoderPositionPeriodMs(523)
        .primaryEncoderVelocityPeriodMs(523);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End TransferIOSparkMax Constructor

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    // inputs.colorSensorTripped = colorSensor.getProximity() >= kColorSensorProximityThreshold;
    inputs.colorSensorTripped = false;
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

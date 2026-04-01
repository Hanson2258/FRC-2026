package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Hang IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class HangIOSparkMax implements HangIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;

  public HangIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(kMetersPerRotation / kGearRatio)
        .velocityConversionFactor(kMetersPerRotation / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);

    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motor.setPeriodicFrameTimeout(0);
  } // End HangIOSparkMax Constructor

  @Override
  public void updateInputs(HangIOInputs inputs) {
    double p = SmartDashboard.getNumber("Hang/kP", kP);
    double i = SmartDashboard.getNumber("Hang/kI", kI);
    double d = SmartDashboard.getNumber("Hang/kD", kD);

    if (p != lastP || i != lastI || d != lastD) {
      lastP = p;
      lastI = i;
      lastD = d;

      var sparkMaxConfig = new SparkMaxConfig();
      sparkMaxConfig.closedLoop.p(p).i(i).d(d);
      sparkMaxConfig.signals
          .appliedOutputPeriodMs(kSignalsPeriodMs)
          .busVoltagePeriodMs(kSignalsPeriodMs)
          .outputCurrentPeriodMs(kSignalsPeriodMs)
          .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
          .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);
      motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      motor.setPeriodicFrameTimeout(0);
    }

    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity() / 60.0;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetMeters) {
    closedLoopController.setSetpoint(targetMeters, SparkBase.ControlType.kPosition);
  } // End setTargetPosition

  @Override
  public void resetEncoders() {
    encoder.setPosition(kStoredPositionMeters);
  } // End resetEncoders

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

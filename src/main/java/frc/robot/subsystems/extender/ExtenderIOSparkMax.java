package frc.robot.subsystems.extender;

import static frc.robot.subsystems.extender.ExtenderConstants.*;

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

/** Extender IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class ExtenderIOSparkMax implements ExtenderIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;

  public ExtenderIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(kIdleMode);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig.smartCurrentLimit(kSmartCurrentLimitAmps);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD).maxOutput(0.3).minOutput(-0.3);
    sparkMaxConfig.signals
        .appliedOutputPeriodMs(kSignalsPeriodMs)
        .busVoltagePeriodMs(kSignalsPeriodMs)
        .outputCurrentPeriodMs(kSignalsPeriodMs)
        .primaryEncoderPositionPeriodMs(kSignalsPeriodMs)
        .primaryEncoderVelocityPeriodMs(kEncoderVelocitySignalPeriodMs);

    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
  public void resetEncoders() {
    encoder.setPosition(Units.radiansToRotations(kMaxRad));
  } // End resetEncoders

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

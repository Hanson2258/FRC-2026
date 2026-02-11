package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/** Intake IO using a single SPARK MAX (NEO 550) with onboard velocity control. */
public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  public IntakeIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    var sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    sparkMaxConfig.inverted(kMotorInverted);
    sparkMaxConfig
        .encoder
        .positionConversionFactor(1.0 / kGearRatio)
        .velocityConversionFactor(1.0 / kGearRatio);
    sparkMaxConfig.closedLoop.p(kP).i(kI).d(kD);
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End IntakeIOSparkMax Constructor

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    double rawRpm = encoder.getVelocity();
    inputs.velocityRadsPerSec =
        (kMotorInverted ? -1.0 : 1.0)
            * Units.rotationsPerMinuteToRadiansPerSecond(rawRpm);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    double setpointRpm =
        Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadsPerSec);
    closedLoopController.setSetpoint(setpointRpm, SparkBase.ControlType.kVelocity);
  } // End setTargetVelocity

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

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

/** Turret IO using a single SPARK MAX (NEO 550) with onboard position control. */
public class TurretIOSparkMax implements TurretIO {

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  public TurretIOSparkMax() {
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
    motor.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  } // End TurretIOSparkMax Constructor

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    setTargetPosition(targetRads, 0.0);
  } // End setTargetPosition

  @Override
  public void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    double targetRot = Units.radiansToRotations(targetRads);
    closedLoopController.setSetpoint(targetRot, SparkBase.ControlType.kPosition);
    // SPARK MAX position control does not expose velocity feedforward; ignore
  } // End setTargetPosition

  @Override
  public void stop() {
    motor.set(0.0);
  } // End stop
}

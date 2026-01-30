package frc.robot.subsystems.shooter.turrent;

import static frc.robot.subsystems.shooter.turrent.TurretConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/** Turret IO implementation using a single SPARK MAX (built-in encoder). */
public class TurretIOSparkMax implements TurretIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public TurretIOSparkMax() {
    motor = new SparkMax(kMotorId, SparkLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();

    var config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = motor.getLastError() == REVLibError.kOk;
    inputs.positionRads =
        Units.rotationsToRadians(encoder.getPosition()) / kGearRatio;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(encoder.getVelocity() / 60.0) / kGearRatio;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}

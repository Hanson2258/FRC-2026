package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearRatio;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Flywheel IO for simulation; uses software velocity control (no onboard controller in sim). */
public class FlywheelIOSim implements FlywheelIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getKrakenX44Foc(1); // TODO: Add correct motor model
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 1.0 / kGearRatio),
          motorModel);

  private double targetVelocityRadsPerSec = 0.0;
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (isStopped) {
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
    } else {
      double currentVelocityRadsPerSec =
          motorSim.getAngularVelocityRadPerSec() / kGearRatio;
      double errorRadsPerSec = targetVelocityRadsPerSec - currentVelocityRadsPerSec;
      double errorRotPerSec = errorRadsPerSec / (2.0 * Math.PI);
      double output = kP * errorRotPerSec;
      appliedVolts = MathUtil.clamp(output, -kMaxVoltage, kMaxVoltage);
      motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    }
    motorSim.update(kLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.velocityRadsPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
  } // End updateInputs

  @Override
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    this.targetVelocityRadsPerSec = targetVelocityRadsPerSec;
    isStopped = false;
  } // End setTargetVelocity

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}

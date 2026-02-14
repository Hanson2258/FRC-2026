package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearRatio;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Flywheel IO for simulation; software velocity PIDF. */
public class FlywheelIOSim implements FlywheelIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getKrakenX44Foc(1); // TODO: Add correct motor model
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 1.0 / kGearRatio),
          motorModel);

  private double targetVelocityRadsPerSec = 0.0;
  private double integralRotSec = 0.0;
  private double previousRotPerSec = 0.0;
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double p = SmartDashboard.getNumber("Flywheel/kP", FlywheelConstants.kP);
    double i = SmartDashboard.getNumber("Flywheel/kI", FlywheelConstants.kI);
    double d = SmartDashboard.getNumber("Flywheel/kD", FlywheelConstants.kD);
    double v = SmartDashboard.getNumber("Flywheel/kV", FlywheelConstants.kV);
    double s = SmartDashboard.getNumber("Flywheel/kS", FlywheelConstants.kS);

    if (isStopped) {
      integralRotSec = 0.0;
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
      previousRotPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio / (2.0 * Math.PI);
    } else {
      double currentVelocityRadsPerSec =
          motorSim.getAngularVelocityRadPerSec() / kGearRatio;
      double targetRotPerSec = targetVelocityRadsPerSec / (2.0 * Math.PI);
      double currentRotPerSec = currentVelocityRadsPerSec / (2.0 * Math.PI);
      double errorRotPerSec = targetRotPerSec - currentRotPerSec;

      integralRotSec += errorRotPerSec * kLoopPeriodSecs;
      double maxIntegralSec = i > 1e-9 ? kMaxVoltage / i : 1e6;
      integralRotSec = MathUtil.clamp(integralRotSec, -maxIntegralSec, maxIntegralSec);

      double velocityErrorDerivativeRotPerSecSq =
          -(currentRotPerSec - previousRotPerSec) / kLoopPeriodSecs;
      previousRotPerSec = currentRotPerSec;

      double staticFf = Math.signum(targetRotPerSec) * s;
      double velocityFf = v * targetRotPerSec;
      double pidOutput =
          p * errorRotPerSec + i * integralRotSec + d * velocityErrorDerivativeRotPerSecSq;
      appliedVolts = MathUtil.clamp(staticFf + velocityFf + pidOutput, -kMaxVoltage, kMaxVoltage);
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

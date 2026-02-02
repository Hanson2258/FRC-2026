package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.kGearRatio;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kP;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kI;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kD;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Hood IO for simulation; uses software PID (no onboard controller in sim). */
public class HoodIOSim implements HoodIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getNeo550(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 100.0 / kGearRatio),
          motorModel);

  private double targetPositionRad = 0.0;
  private double integralRadSec = 0.0;
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (isStopped) {
      integralRadSec = 0.0;
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
    } else {
      double currentPositionRad = motorSim.getAngularPositionRad() / kGearRatio;
      double velocityRadPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio;
      double positionErrorRad = targetPositionRad - currentPositionRad;

      integralRadSec += positionErrorRad * kLoopPeriodSecs;
      double integralRotationsSec = integralRadSec / (2 * Math.PI);
      @SuppressWarnings("unused")
      double maxIntegralRotationsSec = kI > 1e-9 ? kMaxVoltage / kI : Double.MAX_VALUE;
      integralRotationsSec = MathUtil.clamp(integralRotationsSec, -maxIntegralRotationsSec, maxIntegralRotationsSec);
      integralRadSec = integralRotationsSec * (2 * Math.PI);

      double positionErrorRotations = positionErrorRad / (2 * Math.PI);
      double velocityRotationsPerSec = velocityRadPerSec / (2 * Math.PI);
      double pidOutputVolts =
          kP * positionErrorRotations
              + kI * integralRotationsSec
              - kD * velocityRotationsPerSec;
      appliedVolts = MathUtil.clamp(pidOutputVolts, -kMaxVoltage, kMaxVoltage);

      motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    }
    motorSim.update(kLoopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = motorSim.getAngularPositionRad() / kGearRatio;
    inputs.velocityRadsPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    targetPositionRad = targetRads;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}

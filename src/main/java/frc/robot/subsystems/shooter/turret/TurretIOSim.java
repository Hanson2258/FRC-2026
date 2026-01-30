package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kGearRatio;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kP;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kI;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kD;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Turret IO for simulation; uses software PID (no onboard controller in sim). */
public class TurretIOSim implements TurretIO {

  private static final double kLoopPeriodSecs = 0.02;

  private final DCMotor motorModel = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.001, 100.0 / kGearRatio),
          motorModel);

  private double targetPositionRad = 0.0;
  private double integralRadSec = 0.0;
  private double appliedVolts = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (isStopped) {
      integralRadSec = 0.0;
      appliedVolts = 0.0;
      motorSim.setInputVoltage(0.0);
    } else {
      double currentPositionRad = motorSim.getAngularPositionRad() / kGearRatio;
      double velocityRadPerSec = motorSim.getAngularVelocityRadPerSec() / kGearRatio;
      double positionErrorRad = MathUtil.angleModulus(targetPositionRad - currentPositionRad);

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

    // Fill logged inputs for AdvantageKit
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

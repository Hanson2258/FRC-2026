package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;

/** Turret IO for simulation; rate-limited setpoint following. */
public class TurretIOSim implements TurretIO {

  private static final double kLoopPeriodSecs = 0.02;
  /** Max turret rate so that 1 rad takes ~0.3s. */
  private static final double kMaxRadPerSec = 1.0 / 0.2;

  private double targetPositionRad = 0.0;
  private double currentPositionRad = 0.0;
  private boolean isStopped = false;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (!isStopped) {
      double errorRad = MathUtil.angleModulus(targetPositionRad - currentPositionRad);
      double maxStepRad = kMaxRadPerSec * kLoopPeriodSecs;
      double stepRad = MathUtil.clamp(errorRad, -maxStepRad, maxStepRad);
      currentPositionRad += stepRad;
      inputs.velocityRadsPerSec = stepRad / kLoopPeriodSecs;
    } else {
      inputs.velocityRadsPerSec = 0.0;
    }

    inputs.motorConnected = true;
    inputs.positionRads = currentPositionRad;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
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

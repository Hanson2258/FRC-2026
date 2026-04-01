package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/** Hood IO for simulation; slew-rate-limited setpoint following. */
public class HoodIOSim implements HoodIO {

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(kSimMaxSlewRadPerSec);

  private double targetPositionRad = 0.0;
  private double limitedPositionRad = kDisabledAngleRad;
  private boolean isStopped = false;

  public HoodIOSim() {
    slewRateLimiter.reset(kDisabledAngleRad);
  } // End HoodIOSim Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (!isStopped) {
      double clampedTarget = MathUtil.clamp(targetPositionRad, kMinAngleRad, kMaxAngleRad);
      limitedPositionRad = slewRateLimiter.calculate(clampedTarget);
    }

    inputs.motorConnected = true;
    inputs.positionRads = limitedPositionRad;
    inputs.velocityRadsPerSec = 0.0;
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetPositionRad) {
    this.targetPositionRad = targetPositionRad;
    isStopped = false;
  } // End setTargetPosition

  @Override
  public void resetEncoder() {
    limitedPositionRad = kDisabledAngleRad;
    targetPositionRad = kDisabledAngleRad;
    slewRateLimiter.reset(kDisabledAngleRad);
  } // End resetEncoder

  @Override
  public void stop() {
    isStopped = true;
  } // End stop
}

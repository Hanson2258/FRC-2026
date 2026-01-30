package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the turret (one motor + encoder). */
public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Set the target position (radians, encoder frame).
   * Controller runs PID on-device at higher rate than robot loop.
   */
  default void setTargetPosition(double targetRads) {}

  /** Stop the turret (no power). */
  default void stop() {}
}

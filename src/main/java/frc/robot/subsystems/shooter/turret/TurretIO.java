package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Turret (one motor, position controlled). */
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

  /** Set the target position. */
  default void setTargetPosition(double targetRads) {}

  /**
   * Set target position and velocity feedforward (e.g. -robot omega for spin compensation).
   * Default ignores velocity; override in IO implementations that support it.
   */
  default void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    setTargetPosition(targetRads);
  }

  /** Stop the motor (coast). */
  default void stop() {}
}

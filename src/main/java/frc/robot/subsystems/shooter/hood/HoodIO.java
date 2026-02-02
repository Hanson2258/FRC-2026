package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the hood (one motor + encoder). */
public interface HoodIO {

  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Set the target position (radians, encoder frame).
   * Controller runs PID on-device at higher rate than robot loop.
   */
  default void setTargetPosition(double targetRads) {}

  /** Stop the hood (no power). */
  default void stop() {}
}

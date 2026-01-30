package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the flywheel (one motor + encoder, velocity control). */
public interface FlywheelIO {

  @AutoLog
  class FlywheelIOInputs {
    public boolean motorConnected = false;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Set the target velocity (rad/s, flywheel output).
   * Controller runs velocity PID on-device at higher rate than robot loop.
   */
  default void setTargetVelocity(double targetVelocityRadsPerSec) {}

  /** Stop the flywheel (no power). */
  default void stop() {}
}

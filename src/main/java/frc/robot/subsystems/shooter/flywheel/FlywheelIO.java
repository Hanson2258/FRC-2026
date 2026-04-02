package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Flywheel (one motor, velocity controlled). */
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

  /** Set the target velocity. */
  default void setTargetVelocity(double targetVelocityRadPerSec) {}

  /** Stop the motor. */
  default void stop() {}
}

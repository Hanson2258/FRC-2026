package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the intake (one motor, velocity controlled). */
public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = false;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the target velocity (rad/s, output shaft). */
  default void setTargetVelocity(double targetVelocityRadsPerSec) {}

  /** Stop the motor (coast). */
  default void stop() {}
}

package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Hang (one motor, position controlled). */
public interface HangIO {

  @AutoLog
  class HangIOInputs {
    public boolean motorConnected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(HangIOInputs inputs) {}

  /** Set the target position in meters. */
  default void setTargetPosition(double targetMeters) {}

  /** Sets the encoder position (implementation-defined). */
  default void resetEncoders() {}

  /** Stop the motor. */
  default void stop() {}
}

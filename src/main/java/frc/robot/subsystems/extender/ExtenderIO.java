package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Extender (one motor, position controlled). */
public interface ExtenderIO {

  @AutoLog
  class ExtenderIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(ExtenderIOInputs inputs) {}

  /** Set the target position. */
  default void setTargetPosition(double targetRads) {}

  /** Sets the encoder position (implementation-defined). */
  default void resetEncoders() {}

  /** Stop the motor. */
  default void stop() {}
}

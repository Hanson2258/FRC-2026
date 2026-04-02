package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Intake (one motor, voltage controlled). */
public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the motor output voltage, and ignore limits if ignoreLimits is true. */
  default void setVoltage(double volts, boolean ignoreLimits) {}

  /** Stop the motor. */
  default void stop() {}
}

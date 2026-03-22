package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Agitator (one motor, voltage controlled). */
public interface AgitatorIO {

  @AutoLog
  class AgitatorIOInputs {
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(AgitatorIOInputs inputs) {}

  /** Set the motor output voltage. */
  default void setVoltage(double volts) {}

  /** Stop the motor. */
  default void stop() {}
}

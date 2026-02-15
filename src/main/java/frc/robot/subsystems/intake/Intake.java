package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: one motor, voltage controlled (run or coast). */
public class Intake extends SubsystemBase {

  /** Intake mode: idle, intaking (pull in), or reversing (spit out). */
  public enum Mode {
    IDLE,
    INTAKING,
    REVERSING
  }

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs intakeInputs = new IntakeIO.IntakeIOInputs();

  private Mode mode = Mode.IDLE;
  private double targetVoltage = kIdleVoltage;

  public Intake(IntakeIO io) {
    intakeIO = io;
  } // End Intake Constructor

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.recordOutput("Subsystems/Intake/Inputs/MotorConnected", intakeInputs.motorConnected);
    Logger.recordOutput("Subsystems/Intake/Inputs/TargetVolts", getTargetVoltage());
    Logger.recordOutput("Subsystems/Intake/Inputs/AppliedVolts", intakeInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Intake/Inputs/SupplyCurrentAmps", intakeInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Intake/Mode", mode.name());

    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      return;
    }

    // Set the Intake voltage based on the current mode
    switch (mode) {
      case IDLE:
        intakeIO.stop();
        break;
      case INTAKING:
      case REVERSING:
        intakeIO.setVoltage(targetVoltage);
        break;
      default:
        intakeIO.stop();
        break;
    }
  } // End periodic

  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleMode

  /** Set mode to intaking (pull in at intaking voltage). */
  public void setIntakingMode() {
    mode = Mode.INTAKING;
    targetVoltage = kIntakingVoltage;
  } // End setIntakingMode

  /** Set mode to reversing (spit out at reversing voltage). */
  public void setReversingMode() {
    mode = Mode.REVERSING;
    targetVoltage = kReversingVoltage;
  } // End setReversingMode

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode
}

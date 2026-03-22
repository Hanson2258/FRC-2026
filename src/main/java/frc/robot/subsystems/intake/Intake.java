package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: one motor, voltage controlled (run or coast). */
public class Intake extends SubsystemBase {

  /** Intake state: idle, intaking (pull in), or reversing (spit out). */
  public enum State {
    IDLE,
    INTAKING,
    REVERSING
  } // End State enum

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs intakeInputs = new IntakeIO.IntakeIOInputs();

  private State state = State.IDLE;
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
    Logger.recordOutput("Subsystems/Intake/State", state.name());

    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      return;
    }

    // Set the Intake voltage based on the current state
    switch (state) {
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

  /** Set state to idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleState

  /** Set state to intaking (pull in at intaking voltage). */
  public void setIntakingState() {
    state = State.INTAKING;
    targetVoltage = kIntakingVoltage;
  } // End setIntakingState

  /** Set state to reversing (spit out at reversing voltage). */
  public void setReversingState() {
    state = State.REVERSING;
    targetVoltage = kReversingVoltage;
  } // End setReversingState

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  /** Step the target voltage by the given amount. */
  public void stepVoltage(double stepVoltage) {
    if (getState() == State.IDLE) {
      setIntakingState();
      setTargetVoltage(stepVoltage);
    }
    else {
      setTargetVoltage(MathUtil.clamp(getTargetVoltage() + stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (getTargetVoltage() == kIdleVoltage) setIdleState();
  } // End stepVoltage

  /** Current state. */
  public State getState() {
    return state;
  } // End getState
}

package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Agitator subsystem: storage-to-shooter transfer */
public class Agitator extends SubsystemBase {

  /** Agitator state: Idle, Staging (slow pre-load), or Shooting. */
  public enum State {
    IDLE,
    STAGING,
    SHOOTING
  } // End State enum

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();

  private State state = State.IDLE;
  private double targetVoltage = kIdleVoltage;

  public Agitator(AgitatorIO io) {
    agitatorIO = io;
  } // End Agitator Constructor

  @Override
  public void periodic() {
    agitatorIO.updateInputs(agitatorInputs);
    Logger.recordOutput("Subsystems/Agitator/Inputs/MotorConnected", agitatorInputs.motorConnected);
    Logger.recordOutput("Subsystems/Agitator/Inputs/TargetVolts", getTargetVoltage());
    Logger.recordOutput("Subsystems/Agitator/Inputs/AppliedVolts", agitatorInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Agitator/Inputs/SupplyCurrentAmps", agitatorInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Agitator/State", state.name());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      return;
    }

    // Set the Agitator voltage based on the current state
    switch (state) {
      case IDLE:
        agitatorIO.stop();
        break;
      case STAGING:
      case SHOOTING:
        agitatorIO.setVoltage(targetVoltage);
        break;
      default:
        agitatorIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleState

  /** Set state to Staging (slow pre-load). */
  public void setStagingState() {
    state = State.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingState

  /** Set state to Shooting (fast loading). */
  public void setShootingState() {
    state = State.SHOOTING;
    targetVoltage = kShootingVoltage;
  } // End setShootingState

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Step the target voltage by the given amount. */
  public void stepVoltage(double stepVoltage) {
    if (getState() == State.IDLE) {
      setStagingState();
      setTargetVoltage(stepVoltage);
    }
    else {
      setTargetVoltage(MathUtil.clamp(getTargetVoltage() + stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (getTargetVoltage() == kIdleVoltage) setIdleState();
  } // End stepVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  /** Current state. */
  public State getState() {
    return state;
  } // End getState
}

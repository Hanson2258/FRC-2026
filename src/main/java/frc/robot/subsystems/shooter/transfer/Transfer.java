package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Transfer subsystem: Staging (low voltage, stop when sensor tripped (optional)) or Shooting (high voltage). */
public class Transfer extends SubsystemBase {

  /** Transfer state: Idle, Staging (slow pre-load), or Shooting. */
  public enum State {
    IDLE,
    STAGING,
    SHOOTING
  } // End State enum

  private final TransferIO transferIO;
  private final TransferIO.TransferIOInputs transferInputs = new TransferIO.TransferIOInputs();

  private State state = State.IDLE;
  private boolean colourSensorEnabled = false;
  private boolean ballStaged = false;
  private double targetVoltage = kIdleVoltage;

  public Transfer(TransferIO io) {
    transferIO = io;
  } // End Transfer Constructor

  @Override
  public void periodic() {
    transferIO.updateInputs(transferInputs);
    Logger.recordOutput("Subsystems/Shooter/Transfer/Inputs/MotorConnected", transferInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Transfer/Inputs/TargetVolts", getTargetVoltage());
    Logger.recordOutput("Subsystems/Shooter/Transfer/Inputs/AppliedVolts", transferInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Transfer/Inputs/SupplyCurrentAmps", transferInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Shooter/Transfer/Inputs/ColorSensorTripped", transferInputs.colorSensorTripped);
    Logger.recordOutput("Subsystems/Shooter/Transfer/State", state.name());
    Logger.recordOutput("Subsystems/Shooter/Transfer/BallStaged", ballStaged);

    if (DriverStation.isDisabled()) {
      transferIO.stop();
      return;
    }

    // Set the Transfer voltage based on the current state
    switch (state) {
      case IDLE:
        transferIO.stop();
        break;
      case STAGING:
        if (colourSensorEnabled && transferInputs.colorSensorTripped) {
          transferIO.stop();
          ballStaged = true;
          state = State.IDLE;
        } else {
          transferIO.setVoltage(targetVoltage);
        }
        break;
      case SHOOTING:
        transferIO.setVoltage(targetVoltage);
        break;
      default:
        transferIO.stop();
        break;
    }
  } // End periodic
  
  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleState

  /** Set state to Staging (slow pre-load; stop when colour sensor tripped (optional)). */
  public void setStagingState() {
    state = State.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingState

  /** Set state to Shooting (fast load); clears ballStaged. */
  public void setShootingState() {
    state = State.SHOOTING;
    ballStaged = false;
    targetVoltage = kShootingVoltage;
  } // End setShootingState

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
      setStagingState();
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

  /** True when in Staging and colour sensor was tripped (ball at transfer). */
  public boolean isBallStaged() {
    return ballStaged;
  } // End isBallStaged
}

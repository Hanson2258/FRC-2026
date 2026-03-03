package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.RotatorConstants.kAtTargetPositionTolerance;
import static frc.robot.subsystems.extender.RotatorConstants.kDownExtenderRads;
import static frc.robot.subsystems.extender.RotatorConstants.kUpExtenderRads;

public class Rotator extends SubsystemBase {

  /** Extender Mode:
   * IDLE = stop motor
   * UP = set to up position
   * DOWN = set to down position
   */
  public enum Mode {
    IDLE,
    UP,
    DOWN
    }

  private final RotatorIO extenderIO;
  private final RotatorIO.ExtenderIOInputs extenderInputs = new RotatorIO.ExtenderIOInputs();
  
  private Mode state = Mode.IDLE;

  public Rotator(RotatorIO io) {
    extenderIO = io; 
  }

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput("Subsystems/Extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    // TODO: add more inputs

    if (DriverStation.isDisabled()) {
      extenderIO.stop();
      return;
    }

    // Set extender position based on current state
    switch (state) {
      case IDLE:
        extenderIO.stop();
        break;
      case UP:
      case DOWN:
        setTargetRads(extenderInputs.targetPositionRads);
        break;
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to idle (Stay at position) */
  public void setIdleMode() {
    state = Mode.IDLE;
  } // End setIdleState

  /** Set state to up (Go to up position) */
  public void setUpMode() {
    state = Mode.UP;
    setTargetRads(kUpExtenderRads);
  } // End setUpState

  /** Set state to down (Go to down position) */
  public void setDownMode() {
    state = Mode.DOWN;
    setTargetRads(kDownExtenderRads);
  } // End setDownState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetRads(0);
  } // End resetEncoders

  /** Set the target rads, used in UP/DOWN mode */
  public void setTargetRads(double rads) {
    extenderInputs.targetPositionRads = rads;
  } // End setTargetPosition

  /** Returns the target rads */
  public double getTargetRads() {
    return extenderInputs.targetPositionRads;
  } // End getTargetPosition

  /** Get the motors current rads */
  public double getRads() {
    return extenderInputs.positionRads;
  } // End getPosition

  /** Increases the target rads by "steps" */
  public void stepPosition(double steps) {
    setTargetRads(extenderInputs.targetPositionRads + steps);
  } // End stepPosition

  /** Whether the extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getRads() - getTargetRads()) <= kAtTargetPositionTolerance;
  } // End atTargetPosition
}

package frc.robot.subsystems.rotator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.rotator.RotatorConstants.kAtTargetRadsTolerance;
import static frc.robot.subsystems.rotator.RotatorConstants.kD;
import static frc.robot.subsystems.rotator.RotatorConstants.kDownRotatorRads;
import static frc.robot.subsystems.rotator.RotatorConstants.kI;
import static frc.robot.subsystems.rotator.RotatorConstants.kP;
import static frc.robot.subsystems.rotator.RotatorConstants.kUpRotatorRads;

public class Rotator extends SubsystemBase {

  /** Rotator Modes:
   * IDLE = stop motor
   * UP = set to target position (Usually UP position)
   * DOWN = set to target position (Usually DOWN position)
   */
  public enum Mode {
    IDLE,
    UP,
    DOWN
  }

  private final RotatorIO rotatorIO;
  private final RotatorIO.ExtenderIOInputs rotatorInputs = new RotatorIO.ExtenderIOInputs();
  
  private Mode state = Mode.IDLE;

  public Rotator(RotatorIO io) {
    rotatorIO = io; 

    SmartDashboard.putNumber("Flywheel/kP", kP);
    SmartDashboard.putNumber("Flywheel/kI", kI);
    SmartDashboard.putNumber("Flywheel/kD", kD);
  }

  @Override
  public void periodic() {
    rotatorIO.updateInputs(rotatorInputs);
    Logger.recordOutput("Subsystems/Rotator/Inputs/MotorConnected", rotatorInputs.motorConnected);
    Logger.recordOutput("Subsystems/Rotator/Inputs/AppliedVolts", rotatorInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Rotator/Inputs/SupplyCurrentAmps", rotatorInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Rotator/Inputs/TargetPositionRads", rotatorInputs.targetPositionRads);
    Logger.recordOutput("Subsystems/Rotator/Inputs/PositionRads", rotatorInputs.positionRads);
    Logger.recordOutput("Subsystems/Rotator/Inputs/VelocityRadsPerSec", rotatorInputs.velocityRadsPerSec);

    if (DriverStation.isDisabled()) {
      rotatorIO.stop();
      return;
    }

    // Set extender position based on current state
    switch (state) {
      case IDLE:
        rotatorIO.stop();
        break;
      case UP:
      case DOWN:
        setTargetRads(rotatorInputs.targetPositionRads);
        break;
      default:
        rotatorIO.stop();
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
    setTargetRads(kUpRotatorRads);
  } // End setUpState

  /** Set state to down (Go to down position) */
  public void setDownMode() {
    state = Mode.DOWN;
    setTargetRads(kDownRotatorRads);
  } // End setDownState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    rotatorIO.stop();
    rotatorIO.resetEncoders();
    setTargetRads(0);
  } // End resetEncoders

  /** Set the target rads, used in UP/DOWN mode */
  public void setTargetRads(double rads) {
    rotatorInputs.targetPositionRads = rads;
  } // End setTargetPosition

  /** Returns the target rads */
  public double getTargetRads() {
    return rotatorInputs.targetPositionRads;
  } // End getTargetPosition

  /** Get the motors current rads */
  public double getRads() {
    return rotatorInputs.positionRads;
  } // End getPosition

  /** Increases the target rads by "steps" */
  public void stepPosition(double steps) {
    setTargetRads(rotatorInputs.targetPositionRads + steps);
  } // End stepPosition

  /** Whether the extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getRads() - getTargetRads()) <= kAtTargetRadsTolerance;
  } // End atTargetPosition
}

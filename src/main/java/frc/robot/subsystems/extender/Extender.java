package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.ExtenderConstants.kAtTargetPositionTolerance;
import static frc.robot.subsystems.extender.ExtenderConstants.kDownExtenderRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kUpExtenderRads;

public class Extender extends SubsystemBase {

  public enum ExtenderState {
    IDLE,
    UP,
    DOWN
  }

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  
  private ExtenderState state = ExtenderState.IDLE;

  public Extender(ExtenderIO io) {
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

    switch (state) {
      case IDLE:
        extenderIO.stop();
        break;
      case UP:
        if (getTargetPosition() != kUpExtenderRads) {
          setTargetPosition(kUpExtenderRads);
        }
        break;
      case DOWN:
        if (getTargetPosition() != kDownExtenderRads) {
          setTargetPosition(kDownExtenderRads);
        }
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to idle (Stay at position) */
  public void setIdleState() {

  } // End setIdleState

  /** Set state to up (Go to up position) */
  public void setUpState() {
    
  } // End setUpState

  /** Set state to down (Go to down position) */
  public void setDownState() {

  } // End setDownState

  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetPosition(0);
  } // End resetEncoders

  public void setTargetPosition(double position) {
    extenderInputs.targetPositionRads = position;
  } // End setTargetPosition

  public double getTargetPosition() {
    return extenderInputs.targetPositionRads;
  } // End getTargetPosition

  public double getPosition() {
    return extenderInputs.positionRads;
  } // End getPosition

  public void stepPosition(double steps) {
    setTargetPosition(extenderInputs.targetPositionRads + steps);
  } // End stepPosition

  public boolean atTargetPosition() {
    return Math.abs(getPosition() - getTargetPosition()) <= kAtTargetPositionTolerance;
  }
}

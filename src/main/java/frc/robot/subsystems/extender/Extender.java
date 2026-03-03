package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.ExtenderConstants.kAtTargetRadsTolerance;
import static frc.robot.subsystems.extender.ExtenderConstants.kD;
import static frc.robot.subsystems.extender.ExtenderConstants.kDownExtenderDegrees;
import static frc.robot.subsystems.extender.ExtenderConstants.kI;
import static frc.robot.subsystems.extender.ExtenderConstants.kP;
import static frc.robot.subsystems.extender.ExtenderConstants.kUpExtenderDegrees;

public class Extender extends SubsystemBase {

  /** extender states:
   * RETRACTED = extender is facing up
   * EXTENDED = extender is facing forward. If the extender is at the position it will turn off all power and rest on the bumpers
   */
  public enum ExtenderState {
    RETRACTED,
    EXTENDED
  }

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  
  private ExtenderState state = ExtenderState.RETRACTED;

  public Extender(ExtenderIO io) {
    extenderIO = io; 

    SmartDashboard.putNumber("Extender/kP", kP);
    SmartDashboard.putNumber("Extender/kI", kI);
    SmartDashboard.putNumber("Extender/kD", kD);
  }

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput("Subsystems/Extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    Logger.recordOutput("Subsystems/Extender/Inputs/AppliedVolts", extenderInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Extender/Inputs/SupplyCurrentAmps", extenderInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Extender/Inputs/TargetPositionRads", extenderInputs.targetPosition);
    Logger.recordOutput("Subsystems/Extender/Inputs/PositionRads", extenderInputs.position);
    Logger.recordOutput("Subsystems/Extender/Inputs/VelocityRadsPerSec", extenderInputs.velocityRadsPerSec);

    if (DriverStation.isDisabled()) {
      extenderIO.stop();
      return;
    }

    // Set extender position based on current state
    switch (state) {
      case RETRACTED:
      case EXTENDED:
        setTargetPosition(extenderInputs.targetPosition);
        break;
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to retracted state (Go to up position) */
  public void setRetractedState() {
    state = ExtenderState.RETRACTED;
    setTargetPosition(kUpExtenderDegrees);
  } // End setIdleState

  /** Set state to extended state (Go to down position and rest on bumpers) */
  public void setExtendedState() {
    state = ExtenderState.EXTENDED;
    setTargetPosition(kDownExtenderDegrees);
  } // End setUpState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetPosition(0);
  } // End resetEncoders

  /** Set the target degrees, used in RETRACTED/EXTENDED state */
  public void setTargetPosition(double degeres) {
    extenderInputs.targetPosition = degeres;
  } // End setTargetPosition

  /** Returns the target degrees */
  public double getTargetPosition() {
    return extenderInputs.targetPosition;
  } // End getTargetPosition

  /** Get the motors current degrees */
  public double getPosition() {
    return extenderInputs.position;
  } // End getPosition

  /** Increases the target degrees by "steps" */
  public void stepPosition(double steps) {
    setTargetPosition(extenderInputs.targetPosition + steps);
  } // End stepPosition

  /** Whether the extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getPosition() - getTargetPosition()) <= kAtTargetRadsTolerance;
  } // End atTargetPosition
}

package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.ExtenderConstants.kAtTargetRadsTolerance;
import static frc.robot.subsystems.extender.ExtenderConstants.kD;
import static frc.robot.subsystems.extender.ExtenderConstants.kDownExtenderRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kEncoderResetRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kI;
import static frc.robot.subsystems.extender.ExtenderConstants.kMaxRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kMinRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kP;
import static frc.robot.subsystems.extender.ExtenderConstants.kUpExtenderRads;
import static frc.robot.subsystems.extender.ExtenderConstants.kPartialExtenderRads;

/** Extender subsystem: one motor with onboard position control. */
public class Extender extends SubsystemBase {

  /** Extender state: Retracted (facing up), Extended (facing forward). */
  public enum ExtenderState {
    IDLE,
    RETRACTED,
    PARTIAL,
    EXTENDED
  }

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();
  
  private ExtenderState state = ExtenderState.RETRACTED;
  private double targetPosition = kUpExtenderRads;

  public Extender(ExtenderIO io) {
    extenderIO = io; 

    SmartDashboard.putNumber("Extender/kP", kP);
    SmartDashboard.putNumber("Extender/kI", kI);
    SmartDashboard.putNumber("Extender/kD", kD);
    SmartDashboard.putNumber("Extender/TargetPositionRads", extenderInputs.targetPositionRads);

    targetPosition = kUpExtenderRads;
  }

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput("Subsystems/Extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    Logger.recordOutput("Subsystems/Extender/Inputs/AppliedVolts", extenderInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Extender/Inputs/SupplyCurrentAmps", extenderInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Extender/Inputs/TargetPositionRads", Units.radiansToDegrees(extenderInputs.targetPositionRads));
    Logger.recordOutput("Subsystems/Extender/PositionRads", Units.radiansToDegrees(extenderInputs.positionRads));
    Logger.recordOutput("Subsystems/Extender/VelocityRadsPerSec", extenderInputs.velocityRadsPerSec);
    Logger.recordOutput("Subsystems/Extender/State", state.name());
    Logger.recordOutput("Subsystems/Extender/AtTargetPosition", atTargetPosition());

    if (DriverStation.isDisabled()) {
      extenderIO.stop();
      return;
    }

    // Set Extender position based on current state
    switch (state) {
      case RETRACTED:
      case PARTIAL:
        extenderIO.setTargetPosition(clampTargetPosition(targetPosition));
        break;
      case EXTENDED:
        if (Units.radiansToDegrees(getPosition()) > targetPosition - kAtTargetRadsTolerance) {
          extenderIO.stop();
        } else {
          extenderIO.setTargetPosition(clampTargetPosition(targetPosition));
        }
        break;
      case IDLE:
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to idle state (stop extender) */
  public void setIdleState() {
    state = ExtenderState.IDLE;
    setTargetPosition(getPosition());
    extenderIO.stop();
  } // End setIdleState

  /** Set state to retracted state (Go to up position) */
  public void setRetractedState() {
    state = ExtenderState.RETRACTED;
    setTargetPosition(kUpExtenderRads);
  } // End setRetractedState

  /** Set state to partial state (Go to middle position) */
  public void setPartialState() {
    state = ExtenderState.PARTIAL;
    setTargetPosition(kPartialExtenderRads);
  } // End setPartialState

  /** Set state to extended state (Go to down position and rest on bumpers) */
  public void setExtendedState() {
    state = ExtenderState.EXTENDED;
    setTargetPosition(kDownExtenderRads);
  } // End setUpState

  /** Returns the extenders current state */
  public ExtenderState getState() {
    return state;
  } // End getState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetPosition(kEncoderResetRads);
  } // End resetEncoders

  /** Set the target rads, used in RETRACTED/EXTENDED state */
  public void setTargetPosition(double rads) {
    System.out.println("Setting target pos to: " + rads);
    targetPosition = clampTargetPosition(rads);
  } // End setTargetPosition

  /** Returns the target rads */
  public double getTargetPosition() {
    return targetPosition;
  } // End getTargetPosition

  /** Get the motors current rads */
  public double getPosition() {
    return extenderInputs.positionRads;
  } // End getPosition

  /** Step the target Rads by the given amount. */
  public void stepPosition(double steps) {
    setTargetPosition(getTargetPosition() + steps);
  } // End stepPosition

  /** Returns the clamped targetPosition to kMinRads and kMaxRads */
  public double clampTargetPosition(double pos) {
    return MathUtil.clamp(pos, kMinRads, kMaxRads);
  } // End getClampedTargetPos

  /** Whether the extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getPosition() - targetPosition) <= kAtTargetRadsTolerance;
  } // End atTargetPosition
}

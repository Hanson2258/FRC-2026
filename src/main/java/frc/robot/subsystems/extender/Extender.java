package frc.robot.subsystems.extender;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.extender.ExtenderConstants.*;

/** Extender subsystem: one motor with onboard position control. */
public class Extender extends SubsystemBase {

  /** Extender state: Idle, Retracted (facing up), Partial, Extended (facing forward). */
  public enum State {
    IDLE,
    RETRACTED,
    PARTIAL,
    EXTENDED
  } // End State enum

  private final ExtenderIO extenderIO;
  private final ExtenderIO.ExtenderIOInputs extenderInputs = new ExtenderIO.ExtenderIOInputs();

  private State state = State.RETRACTED;
  private double targetPositionRad = kUpExtenderRad;

  public Extender(ExtenderIO io) {
    extenderIO = io;

    SmartDashboard.putNumber("Extender/kP", kP);
    SmartDashboard.putNumber("Extender/kI", kI);
    SmartDashboard.putNumber("Extender/kD", kD);
    SmartDashboard.putNumber("Extender/TargetPositionDeg", Units.radiansToDegrees(extenderInputs.targetPositionRads));

    targetPositionRad = kUpExtenderRad;
  }

  @Override
  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.recordOutput("Subsystems/Extender/Inputs/MotorConnected", extenderInputs.motorConnected);
    Logger.recordOutput("Subsystems/Extender/Inputs/AppliedVolts", extenderInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Extender/Inputs/SupplyCurrentAmps", extenderInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Extender/Inputs/TargetPositionDeg", Units.radiansToDegrees(extenderInputs.targetPositionRads));
    Logger.recordOutput("Subsystems/Extender/PositionDeg", Units.radiansToDegrees(extenderInputs.positionRads));
    Logger.recordOutput("Subsystems/Extender/VelocityDegPerSec", Units.radiansToDegrees(extenderInputs.velocityRadsPerSec));
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
        extenderIO.setTargetPosition(clampTargetPosition(targetPositionRad));
        break;
      case EXTENDED:
        if (getPositionRad() > targetPositionRad - kAtTargetToleranceRad) {
          extenderIO.stop();
        } else {
          extenderIO.setTargetPosition(clampTargetPosition(targetPositionRad));
        }
        break;
      case IDLE:
      default:
        extenderIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (stop Extender). */
  public void setIdleState() {
    state = State.IDLE;
    setTargetPositionRad(getPositionRad());
    extenderIO.stop();
  } // End setIdleState

  /** Set state to Retracted (go to up position). */
  public void setRetractedState() {
    state = State.RETRACTED;
    setTargetPositionRad(kUpExtenderRad);
  } // End setRetractedState

  /** Set state to partial state (Go to middle position) */
  public void setPartialState() {
    state = State.PARTIAL;
    setTargetPositionRad(kPartialExtenderRad);
  } // End setPartialState

  /** Set state to Extended (go to down position and rest on bumpers). */
  public void setExtendedState() {
    state = State.EXTENDED;
    setTargetPositionRad(kExtendedExtenderRad);
  } // End setUpState

  /** Returns the Extender's current state */
  public State getState() {
    return state;
  } // End getState

  /** Sets the motor encoder position to 0 */
  public void resetEncoders() {
    extenderIO.stop();
    extenderIO.resetEncoders();
    setTargetPositionRad(kMaxRad);
  } // End resetEncoders

  /** Set the target position in radians; used in Retracted, Partial, and Extended states. */
  public void setTargetPositionRad(double positionRad) {
    targetPositionRad = clampTargetPosition(positionRad);
  } // End setTargetPositionRad

  /** Returns the target position in radians. */
  public double getTargetPositionRad() {
    return targetPositionRad;
  } // End getTargetPositionRad

  /** Get the motor's current position in radians. */
  public double getPositionRad() {
    return extenderInputs.positionRads;
  } // End getPositionRad

  /** Step position in radians by the given amount. */
  public void stepPositionRad(double deltaRad) {
    setTargetPositionRad(getTargetPositionRad() + deltaRad);
  } // End stepPositionRad

  /** Returns the clamped target angle to kMinRad and kMaxRad */
  public double clampTargetPosition(double positionRad) {
    return MathUtil.clamp(positionRad, kMinRad, kMaxRad);
  } // End getClampedTargetPos

  /** Whether the Extender is at the target position within tolerance */
  public boolean atTargetPosition() {
    return Math.abs(getPositionRad() - targetPositionRad) <= kAtTargetToleranceRad;
  } // End atTargetPosition
}

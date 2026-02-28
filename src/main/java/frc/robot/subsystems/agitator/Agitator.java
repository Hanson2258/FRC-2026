package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Agitator subsystem: storage-to-shooter transfer */
public class Agitator extends SubsystemBase {

  /** Agitator mode: idle, staging (slow pre-load), or shooting. */
  public enum Mode {
    IDLE,
    STAGING,
    SHOOTING
  }

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();
  private Timer dejamTimer = new Timer();
  private boolean isDejamming = false;

  private Mode mode = Mode.IDLE;
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
    Logger.recordOutput("Subsystems/Agitator/Mode", mode.name());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      return;
    }

    // Set the Agitator voltage based on the current mode
    switch (mode) {
      case IDLE:
        agitatorIO.stop();
        break;
      case STAGING:
      case SHOOTING:
        if (dejamTimer.hasElapsed(3) && !isDejamming) {
          agitatorIO.setVoltage(-targetVoltage/2);
          isDejamming = true;
        } else if(isDejamming && dejamTimer.hasElapsed(3.2)) {
          dejamTimer.reset();
          isDejamming = false;
        } else if(!isDejamming) {
          agitatorIO.setVoltage(targetVoltage);
        }
        

        break;
      default:
        agitatorIO.stop();
        break;
    }
  } // End periodic

  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleMode

  /** Set mode to staging (slow pre-load). */
  public void setStagingMode() {
    mode = Mode.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingMode

  /** Set mode to shooting (fast loading). */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
    dejamTimer.reset();
    dejamTimer.start();
    targetVoltage = kShootingVoltage;
  } // End setShootingMode

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Step the target voltage by the given amount. */
  public void stepVoltage(double stepVoltage) {
    if (getMode() == Mode.IDLE) {
      setStagingMode();
      setTargetVoltage(stepVoltage);
    }
    else {
      setTargetVoltage(MathUtil.clamp(getTargetVoltage() + stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (getTargetVoltage() == kIdleVoltage) setIdleMode();
  } // End stepVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode
}

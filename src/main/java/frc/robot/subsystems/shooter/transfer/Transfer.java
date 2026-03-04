package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Transfer subsystem: staging (low voltage, stop when sensor tripped (optional)) or shooting (high voltage). */
public class Transfer extends SubsystemBase {

  /** Transfer mode: idle, staging (slow pre-load), or shooting. */
  public enum Mode {
    IDLE,
    STAGING,
    SHOOTING
  }

  private final TransferIO transferIO;
  private final TransferIO.TransferIOInputs transferInputs = new TransferIO.TransferIOInputs();

  private Mode mode = Mode.IDLE;
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
    Logger.recordOutput("Subsystems/Shooter/Transfer/Mode", mode.name());
    Logger.recordOutput("Subsystems/Shooter/Transfer/BallStaged", ballStaged);

    if (DriverStation.isDisabled()) {
      transferIO.stop();
      return;
    }

    // Set the Transfer voltage based on the current mode
    switch (mode) {
      case IDLE:
        transferIO.stop();
        break;
      case STAGING:
        if (colourSensorEnabled && transferInputs.colorSensorTripped) {
          transferIO.stop();
          ballStaged = true;
          mode = Mode.IDLE;
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
  
  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleMode

  /** Set mode to staging (slow pre-load; stop when colour sensor tripped (optional)). */
  public void setStagingMode() {
    mode = Mode.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingMode

  /** Set mode to shooting (fast load); clears ballStaged. */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
    ballStaged = false;
    targetVoltage = kShootingVoltage;
  } // End setShootingMode

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
    if (getMode() == Mode.IDLE) {
      setStagingMode();
      setTargetVoltage(stepVoltage);
    }
    else {
      setTargetVoltage(MathUtil.clamp(getTargetVoltage() + stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (getTargetVoltage() == kIdleVoltage) setIdleMode();
  } // End stepVoltage

  /** Current mode. */
  public Mode getMode() {
    return mode;
  } // End getMode

  /** True when staging and colour sensor was tripped (ball at transfer). */
  public boolean isBallStaged() {
    return ballStaged;
  } // End isBallStaged
}

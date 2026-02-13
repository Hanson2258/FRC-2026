package frc.robot.subsystems.shooter.transfer;

import static frc.robot.subsystems.shooter.transfer.TransferConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Transfer subsystem: staging (slow + stop when sensor tripped) or shooting (fast). */
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
  private boolean ballStaged = false;

  public Transfer(TransferIO io) {
    transferIO = io;
  }

  @Override
  public void periodic() {
    transferIO.updateInputs(transferInputs);
    Logger.recordOutput("Transfer/Inputs/MotorConnected", transferInputs.motorConnected);
    Logger.recordOutput("Transfer/Inputs/VelocityRadsPerSec", transferInputs.velocityRadsPerSec);
    Logger.recordOutput("Transfer/Inputs/AppliedVolts", transferInputs.appliedVolts);
    Logger.recordOutput("Transfer/Inputs/SupplyCurrentAmps", transferInputs.supplyCurrentAmps);
    Logger.recordOutput("Transfer/Inputs/ColorSensorTripped", transferInputs.colorSensorTripped);
    Logger.recordOutput("Transfer/Mode", mode.name());
    Logger.recordOutput("Transfer/BallStaged", ballStaged);
    Logger.recordOutput("Transfer/VelocityRpm", getVelocityRpm());

    if (DriverStation.isDisabled()) {
      transferIO.stop();
      return;
    }

    switch (mode) {
      case IDLE:
        transferIO.stop();
        break;
      case STAGING:
        if (transferInputs.colorSensorTripped) {
          transferIO.stop();
          ballStaged = true;
          mode = Mode.IDLE;
        } else {
          transferIO.setTargetVelocity(kStagingVelocityRadsPerSec);
        }
        break;
      case SHOOTING:
        transferIO.setTargetVelocity(kShootingVelocityRadsPerSec);
        break;
      default:
        transferIO.stop();
        break;
    }
  }

  /** Set mode to staging (slow velocity; stop when colour sensor tripped). */
  public void setStagingMode() {
    mode = Mode.STAGING;
  }

  /** Set mode to shooting (high velocity); clears ballStaged. */
  public void setShootingMode() {
    mode = Mode.SHOOTING;
    ballStaged = false;
  }

  /** Set mode to idle (motor stopped). */
  public void setIdleMode() {
    mode = Mode.IDLE;
  }

  /** Current mode. */
  public Mode getMode() {
    return mode;
  }

  /** True when staging and colour sensor was tripped (ball at transfer). */
  public boolean isBallStaged() {
    return ballStaged;
  }

  /** Current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return transferInputs.velocityRadsPerSec;
  }

  /** Current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(transferInputs.velocityRadsPerSec);
  }
}

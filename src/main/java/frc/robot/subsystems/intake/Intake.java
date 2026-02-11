package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem: one motor, velocity controlled (run at target speed or coast). */
public class Intake extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs intakeInputs = new IntakeIO.IntakeIOInputs();

  private boolean isRunning = false;

  public Intake(IntakeIO io) {
    intakeIO = io;
  } // End Intake Constructor

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.recordOutput("Intake/Inputs/MotorConnected", intakeInputs.motorConnected);
    Logger.recordOutput("Intake/Inputs/VelocityRadsPerSec", intakeInputs.velocityRadsPerSec);
    Logger.recordOutput("Intake/Inputs/AppliedVolts", intakeInputs.appliedVolts);
    Logger.recordOutput("Intake/Inputs/SupplyCurrentAmps", intakeInputs.supplyCurrentAmps);
    Logger.recordOutput("Intake/VelocityRpm", getVelocityRpm());

    if (DriverStation.isDisabled()) {
      intakeIO.stop();
      return;
    }

    if (isRunning) {
      intakeIO.setTargetVelocity(kTargetVelocityRadsPerSec);
    } else {
      intakeIO.stop();
    }
  } // End periodic

  /** Run the intake at the configured target velocity. */
  public void run() {
    isRunning = true;
  } // End run

  /** Stop the intake (coast). */
  public void stop() {
    isRunning = false;
  } // End stop

  /** Whether the intake is currently commanded to run. */
  public boolean isRunning() {
    return isRunning;
  } // End isRunning

  /** Current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return intakeInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(intakeInputs.velocityRadsPerSec);
  } // End getVelocityRpm
}

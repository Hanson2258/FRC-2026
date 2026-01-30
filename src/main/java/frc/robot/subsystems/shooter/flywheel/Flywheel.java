package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Flywheel subsystem: one motor with onboard velocity control for consistent spin. */
public class Flywheel extends SubsystemBase {

  private final FlywheelIO flywheelIO;
  private final FlywheelIO.FlywheelIOInputs flywheelInputs = new FlywheelIO.FlywheelIOInputs();

  private double targetVelocityRadsPerSec = 0.0;

  public Flywheel(FlywheelIO io) {
    flywheelIO = io;
  } // End Flywheel Constructor

  @Override
  public void periodic() {
    // Update the flywheel inputs and record the values
    flywheelIO.updateInputs(flywheelInputs);
    Logger.recordOutput("Flywheel/Inputs/MotorConnected", flywheelInputs.motorConnected);
    Logger.recordOutput("Flywheel/Inputs/VelocityRadsPerSec", flywheelInputs.velocityRadsPerSec);
    Logger.recordOutput("Flywheel/Inputs/AppliedVolts", flywheelInputs.appliedVolts);
    Logger.recordOutput("Flywheel/Inputs/SupplyCurrentAmps", flywheelInputs.supplyCurrentAmps);
    Logger.recordOutput("Flywheel/VelocityRpm", getVelocityRpm());
    Logger.recordOutput("Flywheel/TargetVelocityRpm", getTargetVelocityRpm());

    if (DriverStation.isDisabled()) {
      flywheelIO.stop();
      return;
    }

    flywheelIO.setTargetVelocity(targetVelocityRadsPerSec);
  } // End periodic

  /** Set the target velocity (rad/s, flywheel output). */
  public void setTargetVelocity(double targetVelocityRadsPerSec) {
    this.targetVelocityRadsPerSec = targetVelocityRadsPerSec;
  } // End setTargetVelocity

  /** Get the current target velocity (rad/s). */
  public double getTargetVelocityRadsPerSec() {
    return targetVelocityRadsPerSec;
  } // End getTargetVelocityRadsPerSec

  /** Get the current target velocity (RPM). */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadsPerSec);
  } // End getTargetVelocityRpm

  /** Get the current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return flywheelInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Get the current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
  } // End getVelocityRpm

  /** Whether the flywheel is at target speed within tolerance. */
  public boolean atSpeed() {
    double currentVelocityRadsPerSec = flywheelInputs.velocityRadsPerSec;
    return Math.abs(currentVelocityRadsPerSec - targetVelocityRadsPerSec)
        <= kAtSpeedToleranceRadsPerSec;
  } // End atSpeed
}

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kAtTargetVelocityToleranceRadsPerSec;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kDefaultTargetVelocityRadsPerSec;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kIdleVelocityRadsPerSec;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;


/** Flywheel subsystem: one motor with onboard velocity control; state machine Idle / Charging / AtSpeed. */
public class Flywheel extends SubsystemBase {

  /** Flywheel state: Idle (low velocity), Charging (ramping to target), AtSpeed (ready to shoot). */
  public enum FlywheelState {
    IDLE,
    CHARGING,
    AT_SPEED
  }

  private final FlywheelIO flywheelIO;
  private final FlywheelIO.FlywheelIOInputs flywheelInputs = new FlywheelIO.FlywheelIOInputs();

  private FlywheelState state = FlywheelState.IDLE;
  private double targetVelocityRadsPerSec = kDefaultTargetVelocityRadsPerSec;
  public Flywheel(FlywheelIO io) {
    flywheelIO = io;
    
    // Publish default gains so SmartDashboard has keys for tuning (values read in IO layer)
    SmartDashboard.putNumber("Flywheel/kP", kP);
    SmartDashboard.putNumber("Flywheel/kI", kI);
    SmartDashboard.putNumber("Flywheel/kD", kD);
    SmartDashboard.putNumber("Flywheel/kV", kV);
    SmartDashboard.putNumber("Flywheel/kS", kS);
    SmartDashboard.putNumber("Flywheel/TargetVelocityRadsPerSec", targetVelocityRadsPerSec);
  } // End Flywheel Constructor

  @Override
  public void periodic() {
    // Read TargetVelocityRadsPerSec from SmartDashboard for live tuning
    targetVelocityRadsPerSec = SmartDashboard.getNumber("Flywheel/TargetVelocityRadsPerSec", kDefaultTargetVelocityRadsPerSec);

    // Update the flywheel inputs and record the values
    flywheelIO.updateInputs(flywheelInputs);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/MotorConnected", flywheelInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/VelocityRadsPerSec", flywheelInputs.velocityRadsPerSec);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/AppliedVolts", flywheelInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/SupplyCurrentAmps", flywheelInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/VelocityRpm", getVelocityRpm());
    Logger.recordOutput("Subsystems/Shooter/Flywheel/TargetVelocityRpm", getTargetVelocityRpm());
    Logger.recordOutput("Subsystems/Shooter/Flywheel/State", state.name());

    // Auto-transition Charging → AtSpeed when at target velocity
    if (state == FlywheelState.CHARGING && atTargetVelocity()) {
      state = FlywheelState.AT_SPEED;
    }

    // If in AtSpeed but velocity fell below/outside tolerance, go back to Charging to re-ramp
    if (state == FlywheelState.AT_SPEED && !atTargetVelocity()) {
      state = FlywheelState.CHARGING;
    }

    double velocityToUse = state == FlywheelState.IDLE ? kIdleVelocityRadsPerSec : targetVelocityRadsPerSec;
    Logger.recordOutput("Subsystems/Shooter/Flywheel/TargetVelocityRadsPerSec", velocityToUse);

    if (DriverStation.isDisabled()) {
      flywheelIO.stop();
      return;
    }
    
    flywheelIO.setTargetVelocity(velocityToUse);
  } // End periodic

  /** Set the flywheel state. */
  public void setState(FlywheelState newState) {
    state = newState;
  } // End setState

  /** Set the target velocity (rad/s) used when state is Charging or AtSpeed. */
  public void setTargetVelocityRadsPerSec(double radsPerSec) {
    targetVelocityRadsPerSec = radsPerSec;
  } // End setTargetVelocityRadsPerSec

  /** Get the current flywheel state. */
  public FlywheelState getState() {
    return state;
  } // End getState

  /** Get the current target velocity (rad/s) for the current state. */
  public double getTargetVelocityRadsPerSec() {
    return state == FlywheelState.IDLE ? kIdleVelocityRadsPerSec : targetVelocityRadsPerSec;
  } // End getTargetVelocityRadsPerSec

  /** Get the current target velocity (RPM). */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(getTargetVelocityRadsPerSec());
  } // End getTargetVelocityRpm

  /** Get the current velocity (rad/s). */
  public double getVelocityRadsPerSec() {
    return flywheelInputs.velocityRadsPerSec;
  } // End getVelocityRadsPerSec

  /** Get the current velocity (RPM). */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
  } // End getVelocityRpm

  /** Step the target velocity by the given amount. */
  public void stepVelocityRadsPerSec(double stepRadsPerSec) {
    setTargetVelocityRadsPerSec(getTargetVelocityRadsPerSec() + stepRadsPerSec);
    setState(FlywheelState.CHARGING);
  } // End stepVelocityRadsPerSec

  /** Whether the flywheel is at target velocity within tolerance. */
  public boolean atTargetVelocity() {
    double currentVelocityRadsPerSec = flywheelInputs.velocityRadsPerSec;
    double targetVelocityRadsPerSec = getTargetVelocityRadsPerSec();
    return Math.abs(currentVelocityRadsPerSec - targetVelocityRadsPerSec)
        <= kAtTargetVelocityToleranceRadsPerSec;
  } // End atTargetVelocity
}

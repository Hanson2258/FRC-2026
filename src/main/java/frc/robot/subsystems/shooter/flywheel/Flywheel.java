package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

/** Flywheel subsystem: one motor with onboard velocity control; state machine Idle / Charging / At Speed. */
public class Flywheel extends SubsystemBase {

  /** Flywheel state: Idle (low velocity), Charging (ramping to target), At_Speed (ready to shoot). */
  public enum State {
    IDLE,
    CHARGING,
    AT_SPEED
  } // End State enum

  private final FlywheelIO flywheelIO;
  private final FlywheelIO.FlywheelIOInputs flywheelInputs = new FlywheelIO.FlywheelIOInputs();

  private State state = State.IDLE;
  private double targetVelocityRadPerSec = kDefaultTargetVelocityRadPerSec;

  private double lastSmartDashboardTargetVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec);

  public Flywheel(FlywheelIO io) {
    flywheelIO = io;

    // Publish default gains so SmartDashboard has keys for tuning (values read in IO layer)
    SmartDashboard.putNumber("Flywheel/kP", kP);
    SmartDashboard.putNumber("Flywheel/kI", kI);
    SmartDashboard.putNumber("Flywheel/kD", kD);
    SmartDashboard.putNumber("Flywheel/kV", kV);
    SmartDashboard.putNumber("Flywheel/kS", kS);
    SmartDashboard.putNumber("Flywheel/TargetVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec));
  } // End Flywheel Constructor

  @Override
  public void periodic() {
    // Read TargetVelocityRadsPerSec from SmartDashboard for live tuning    
    double targetRpm = SmartDashboard.getNumber("Flywheel/TargetVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(kDefaultTargetVelocityRadPerSec));

    if (targetRpm != lastSmartDashboardTargetVelocityRpm) {
      setState(State.CHARGING);
      setTargetVelocityRadPerSec(Units.rotationsPerMinuteToRadiansPerSecond(targetRpm));
    }

    lastSmartDashboardTargetVelocityRpm = targetRpm;

    // Update the Flywheel inputs and record the values
    flywheelIO.updateInputs(flywheelInputs);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/MotorConnected", flywheelInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/VelocityRpm", getVelocityRpm());
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/AppliedVolts", flywheelInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/Inputs/SupplyCurrentAmps", flywheelInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Shooter/Flywheel/VelocityRpm", getVelocityRpm());
    Logger.recordOutput("Subsystems/Shooter/Flywheel/TargetVelocityRpm", getTargetVelocityRpm());
    Logger.recordOutput("Subsystems/Shooter/Flywheel/State", state.name());

    // Auto-transition Charging → At_Speed when at target velocity
    if (state == State.CHARGING && atTargetVelocity()) {
      state = State.AT_SPEED;
    }

    // If in At_Speed but velocity fell below/outside tolerance, go back to Charging to re-ramp
    if (state == State.AT_SPEED && !atTargetVelocity()) {
      state = State.CHARGING;
    }

    double velocityRadPerSecToUse = getTargetVelocityRadPerSec();

    if (DriverStation.isDisabled()) {
      flywheelIO.stop();
      return;
    }

    flywheelIO.setTargetVelocity(velocityRadPerSecToUse);
  } // End periodic

  /** Set the Flywheel state. */
  public void setState(State newState) {
    state = newState;
  } // End setState

  /** Set the target velocity used when state is Charging or At_Speed. */
  public void setTargetVelocityRadPerSec(double targetVelocityRadPerSec) {
    this.targetVelocityRadPerSec = targetVelocityRadPerSec;
    SmartDashboard.putNumber("Flywheel/TargetVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec));
    lastSmartDashboardTargetVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec);
  } // End setTargetVelocityRadPerSec

  /** Get the current Flywheel state. */
  public State getState() {
    return state;
  } // End getState

  /** Get the current target velocity for the current state. */
  public double getTargetVelocityRadPerSec() {
    return state == State.IDLE ? kIdleVelocityRadPerSec : targetVelocityRadPerSec;
  } // End getTargetVelocityRadPerSec

  /** Get the current target velocity. */
  public double getTargetVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(getTargetVelocityRadPerSec());
  } // End getTargetVelocityRpm

  /** Get the current velocity. */
  public double getVelocityRadPerSec() {
    return flywheelInputs.velocityRadsPerSec;
  } // End getVelocityRadPerSec

  /** Get the current measured velocity. */
  public double getVelocityRpm() {
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
  } // End getVelocityRpm

  /** Step the target velocity by the given amount; sets state to Charging. */
  public void stepVelocityRadPerSec(double stepVelocityRadPerSec) {
    setState(State.CHARGING);
    setTargetVelocityRadPerSec(getTargetVelocityRadPerSec() + stepVelocityRadPerSec);
  } // End stepVelocityRadPerSec

  /** Whether the Flywheel is at target velocity within tolerance. */
  public boolean atTargetVelocity() {
    double currentVelocityRadPerSec = flywheelInputs.velocityRadsPerSec;
    double targetVelocityRadPerSec = getTargetVelocityRadPerSec();
    return Math.abs(currentVelocityRadPerSec - targetVelocityRadPerSec)
        <= kAtTargetVelocityToleranceRadPerSec;
  } // End atTargetVelocity
}

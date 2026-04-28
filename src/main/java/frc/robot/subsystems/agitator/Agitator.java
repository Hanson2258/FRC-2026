package frc.robot.subsystems.agitator;

import static frc.robot.subsystems.agitator.AgitatorConstants.*;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Agitator subsystem: storage-to-shooter transfer. */
public class Agitator extends SubsystemBase {
  private static final String kTargetVoltageKey = "Agitator/TargetVoltage";
  private static final double kTargetVoltageWidgetEpsilon = 1e-3;

  /** Agitator state: Idle, Staging (slow pre-load), Shooting, or Manual. */
  public enum State {
    IDLE,
    STAGING,
    SHOOTING,
    MANUAL
  } // End State enum

  private final AgitatorIO agitatorIO;
  private final AgitatorIO.AgitatorIOInputs agitatorInputs = new AgitatorIO.AgitatorIOInputs();
  private final String logRoot;

  private State state = State.IDLE;
  private double targetVoltage = kIdleVoltage;
  private BooleanSupplier ignoreLimitsSupplier = () -> false;
  private double lastTargetVoltageDashboardWrite = Double.NaN;

  public Agitator(AgitatorIO io) {
    this(io, "");
  } // End Agitator Constructor

  public Agitator(AgitatorIO io, String logRoot) {
    agitatorIO = io;
    this.logRoot = logRoot;
    SmartDashboard.putNumber(kTargetVoltageKey, targetVoltage);
  } // End Agitator Constructor

  @Override
  public void periodic() {
    agitatorIO.updateInputs(agitatorInputs);
    Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/MotorConnected", agitatorInputs.motorConnected);
    Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/AppliedVolts", agitatorInputs.appliedVolts);
    Logger.recordOutput(logRoot + "Subsystems/Agitator/Inputs/SupplyCurrentAmps", agitatorInputs.supplyCurrentAmps);
    Logger.recordOutput(logRoot + "Subsystems/Agitator/TargetVolts", getTargetVoltage());
    Logger.recordOutput(logRoot + "Subsystems/Agitator/State", state.name());

    if (DriverStation.isDisabled()) {
      agitatorIO.stop();
      lastTargetVoltageDashboardWrite = Double.NaN;
      return;
    }

    boolean useSmartDashboardTarget = ignoreLimitsSupplier.getAsBoolean();
    if (useSmartDashboardTarget && state != State.IDLE) {
      double dashboardVolts = SmartDashboard.getNumber(kTargetVoltageKey, targetVoltage);
      if (!Double.isNaN(lastTargetVoltageDashboardWrite)
          && Math.abs(dashboardVolts - lastTargetVoltageDashboardWrite) > kTargetVoltageWidgetEpsilon) {
        setTargetVoltage(dashboardVolts);
      }
    }
    SmartDashboard.putNumber(kTargetVoltageKey, targetVoltage);
    lastTargetVoltageDashboardWrite = targetVoltage;

    // Set the Agitator voltage based on the current state.
    switch (state) {
      case IDLE:
        agitatorIO.stop();
        break;
      case STAGING:
      case SHOOTING:
      case MANUAL:
        agitatorIO.setVoltage(targetVoltage, ignoreLimitsSupplier.getAsBoolean());
        break;
      default:
        agitatorIO.stop();
        break;
    }
  } // End periodic

  /** Set state to Idle (motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
    targetVoltage = kIdleVoltage;
  } // End setIdleState

  /** Set state to Staging (slow pre-load). */
  public void setStagingState() {
    state = State.STAGING;
    targetVoltage = kStagingVoltage;
  } // End setStagingState

  /** Set state to Shooting (fast loading). */
  public void setShootingState() {
    state = State.SHOOTING;
    targetVoltage = kShootingVoltage;
  } // End setShootingState

  /** Get current state. */
  public State getState() {
    return state;
  } // End getState

  /** Set the target voltage. */
  public void setTargetVoltage(double volts) {
    targetVoltage = volts;
  } // End setTargetVoltage

  /** Get the current target voltage. */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage


  /** Set supplier for ignoring limits. */
  public void setIgnoreLimitsSupplier(BooleanSupplier supplier) {
    ignoreLimitsSupplier = supplier != null ? supplier : () -> false;
  } // End setIgnoreLimitsSupplier

  /** Step the target voltage by the given amount. */
  public void stepVoltage(double stepVoltage) {
    boolean wasIdle = getState() == State.IDLE;
    state = State.MANUAL;
    boolean ignoreLimits = ignoreLimitsSupplier.getAsBoolean();
    if (wasIdle) {
      setTargetVoltage(ignoreLimits 
        ? MathUtil.clamp(stepVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(stepVoltage, -kMaxVoltage, kMaxVoltage));
    }
    else {
      double stepTargetVoltage = getTargetVoltage() + stepVoltage;
      setTargetVoltage(ignoreLimits 
        ? MathUtil.clamp(stepTargetVoltage, -Constants.kNominalVoltage, Constants.kNominalVoltage)
        : MathUtil.clamp(stepTargetVoltage, -kMaxVoltage, kMaxVoltage));
    }
    if (Math.abs(getTargetVoltage() - kIdleVoltage) < 1e-6) {
      setIdleState();
    }
  } // End stepVoltage
}

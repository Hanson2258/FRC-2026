package frc.robot.subsystems.hang;

import static frc.robot.subsystems.hang.HangConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Hang subsystem: extend/retract controlled to potentiometer voltage setpoints (SPARK MAX + analog pot). */
public class Hang extends SubsystemBase {

  /** Hang state: Idle, Stored, Level 1, Manual. */
  public enum State {
    IDLE,
    STORED,
    LEVEL_1,
    MANUAL
  } // End State enum

  private final HangIO hangIO;
  private final HangIO.HangIOInputs hangInputs = new HangIO.HangIOInputs();

  private final PIDController voltageController;

  private State state = State.IDLE;
  private double targetVoltage = kStoredVoltage;

  private double lastP = kP;
  private double lastI = kI;
  private double lastD = kD;

  public Hang(HangIO io) {
    hangIO = io;

    voltageController = new PIDController(kP, kI, kD);
    voltageController.setTolerance(kAtTargetToleranceVolts);

    // SmartDashboard tuning for PID and preset voltages
    SmartDashboard.putNumber("Hang/kP", kP);
    SmartDashboard.putNumber("Hang/kI", kI);
    SmartDashboard.putNumber("Hang/kD", kD);
    SmartDashboard.putNumber("Hang/StoredVoltage", kStoredVoltage);
    SmartDashboard.putNumber("Hang/Level1Voltage", kLevel1Voltage);
  } // End Hang Constructor

  @Override
  public void periodic() {
    hangIO.updateInputs(hangInputs);

    // Update PID gains from SmartDashboard if changed
    double p = SmartDashboard.getNumber("Hang/kP", kP);
    double i = SmartDashboard.getNumber("Hang/kI", kI);
    double d = SmartDashboard.getNumber("Hang/kD", kD);
    if (p != lastP || i != lastI || d != lastD) {
      lastP = p;
      lastI = i;
      lastD = d;
      voltageController.setPID(p, i, d);
    }

    double currentVoltage = getPotVoltage();

    Logger.recordOutput("Subsystems/Hang/Inputs/MotorConnected", hangInputs.motorConnected);
    Logger.recordOutput("Subsystems/Hang/Inputs/AppliedVolts", hangInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Hang/Inputs/SupplyCurrentAmps", hangInputs.supplyCurrentAmps);
    Logger.recordOutput("Subsystems/Hang/PotVoltage", currentVoltage);
    Logger.recordOutput("Subsystems/Hang/TargetVoltage", targetVoltage);
    Logger.recordOutput("Subsystems/Hang/State", state.name());

    if (DriverStation.isDisabled()) {
      hangIO.stop();
      voltageController.reset();
      return;
    }

    if (state == State.IDLE) {
      hangIO.stop();
      voltageController.reset();
      return;
    }

    double clampedTargetVolts = MathUtil.clamp(targetVoltage, kPotExtendedVoltage, kPotRetractedVoltage);
    double outputVolts = -voltageController.calculate(currentVoltage, clampedTargetVolts);
    outputVolts = MathUtil.clamp(outputVolts, -kMaxVoltage, kMaxVoltage);

    hangIO.setVoltage(outputVolts);
  } // End periodic

  /** Current potentiometer voltage (V). */
  public double getPotVoltage() {
    return hangInputs.potVoltage;
  } // End getPotVoltage

  /** Current target potentiometer voltage (V). */
  public double getTargetVoltage() {
    return targetVoltage;
  } // End getTargetVoltage

  /** Current state. */
  public State getState() {
    return state;
  } // End getState

  /** True when pot voltage is at target within tolerance. */
  public boolean atTarget() {
    double clampedTarget = MathUtil.clamp(targetVoltage, kPotExtendedVoltage, kPotRetractedVoltage);
    return Math.abs(getPotVoltage() - clampedTarget) <= kAtTargetToleranceVolts;
  } // End atTarget

  /** Set state to Stored (retracted preset voltage). */
  public void setStoredState() {
    state = State.STORED;
    targetVoltage = SmartDashboard.getNumber("Hang/StoredVoltage", kStoredVoltage);
    voltageController.reset();
  } // End setStoredState

  /** Set state to Level 1 (extended preset voltage). */
  public void setLevel1State() {
    state = State.LEVEL_1;
    targetVoltage = SmartDashboard.getNumber("Hang/Level1Voltage", kLevel1Voltage);
    voltageController.reset();
  } // End setLevel1State

  /** Set state to Manual with the given target potentiometer voltage (V). */
  public void setManualTargetVoltage(double voltageV) {
    state = State.MANUAL;
    targetVoltage = voltageV;
    voltageController.reset();
  } // End setManualTargetVoltage

  /** Step the target voltage by the given amount (V); sets state to Manual. */
  public void stepVolts(double stepVolts) {
    state = State.MANUAL;
    targetVoltage = MathUtil.clamp(getTargetVoltage() + stepVolts, kPotExtendedVoltage, kPotRetractedVoltage);
    voltageController.reset();
  } // End stepVolts

  /** Set state to Idle (no PID output, motor stopped). */
  public void setIdleState() {
    state = State.IDLE;
  } // End setIdleState
}


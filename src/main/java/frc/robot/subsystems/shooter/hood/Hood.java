package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Hood subsystem: position-controlled mechanism that sets shooter angle (radians). */
public class Hood extends SubsystemBase {

  private final HoodIO hoodIO;
  private final HoodIO.HoodIOInputs hoodInputs = new HoodIO.HoodIOInputs();

  private double targetAngleRad = 0.0;

  public Hood(HoodIO io) {
    hoodIO = io;
  } // End Hood Constructor

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    Logger.recordOutput("Hood/Inputs/MotorConnected", hoodInputs.motorConnected);
    Logger.recordOutput("Hood/Inputs/PositionRads", hoodInputs.positionRads);
    Logger.recordOutput("Hood/Inputs/VelocityRadsPerSec", hoodInputs.velocityRadsPerSec);
    Logger.recordOutput("Hood/Inputs/AppliedVolts", hoodInputs.appliedVolts);
    Logger.recordOutput("Hood/Inputs/SupplyCurrentAmps", hoodInputs.supplyCurrentAmps);
    Logger.recordOutput("Hood/PositionDegrees", getAngleRad() * 180.0 / Math.PI);
    Logger.recordOutput("Hood/TargetDegrees", targetAngleRad * 180.0 / Math.PI);

    if (DriverStation.isDisabled()) {
      hoodIO.stop();
      return;
    }

    double clampedRad = MathUtil.clamp(targetAngleRad, kMinAngleRad, kMaxAngleRad);
    double targetPositionRad = clampedRad + kEncoderZeroOffsetRad;
    hoodIO.setTargetPosition(targetPositionRad);
  } // End periodic

  /** Set the target angle (radians). Clamped to min/max in periodic. */
  public void setTargetAngleRad(double angleRad) {
    targetAngleRad = angleRad;
  } // End setTargetAngleRad

  /** Get the current target angle (radians). */
  public double getTargetAngleRad() {
    return targetAngleRad;
  } // End getTargetAngleRad

  /** Get the current hood angle (radians). */
  public double getAngleRad() {
    return hoodInputs.positionRads + kEncoderZeroOffsetRad;
  } // End getAngleRad

  /** Whether the hood is at the target angle within tolerance. */
  public boolean atTarget() {
    double currentRad = hoodInputs.positionRads + kEncoderZeroOffsetRad;
    double targetClamped = MathUtil.clamp(targetAngleRad, kMinAngleRad, kMaxAngleRad);
    return Math.abs(currentRad - targetClamped) <= kAtTargetToleranceRad;
  } // End atTarget
}

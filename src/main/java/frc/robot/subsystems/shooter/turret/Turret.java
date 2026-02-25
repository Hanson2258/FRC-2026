package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem: one motor with onboard position control, aimed at a hub angle. */
public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs();

  private static final double kAtHubToleranceRad = Units.degreesToRadians(2.0);
  private Rotation2d hubAngleRelativeToRobot = Rotation2d.kZero;
  private double velocityFeedforwardRadPerSec = 0.0;

  public Turret(TurretIO io) {
    turretIO = io;
  } // End Turret Constructor

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);
    double targetPositionRad = DriverStation.isDisabled() ? 0.0 : getClampedHubAngleRad() - kEncoderZeroOffsetRad;
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/MotorConnected", turretInputs.motorConnected);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/TargetPositionRads", targetPositionRad);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/PositionRads", turretInputs.positionRads);
    Logger.recordOutput("Subsystems/Shooter/Turret/PositionDegrees", getPosition().getDegrees());
    Logger.recordOutput("Subsystems/Shooter/Turret/HubDegrees", getHubAngleRelativeToRobot().getDegrees());
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/VelocityRadsPerSec", turretInputs.velocityRadsPerSec);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/AppliedVolts", turretInputs.appliedVolts);
    Logger.recordOutput("Subsystems/Shooter/Turret/Inputs/SupplyCurrentAmps", turretInputs.supplyCurrentAmps);

    if (DriverStation.isDisabled()) {
      turretIO.setTargetPosition(0.0, 0.0);
      return;
    }

    turretIO.setTargetPosition(targetPositionRad, velocityFeedforwardRadPerSec);
  } // End periodic

  /** Set velocity feedforward (rad/s) for spin compensation; e.g. -robot omega. */
  public void setVelocityFeedforwardRadPerSec(double radPerSec) {
    velocityFeedforwardRadPerSec = radPerSec;
  } // End setVelocityFeedforwardRadPerSec

  /** Set the hub angle (robot frame: 0 = forward). Clamped to min/max in periodic. */
  public void setHubAngleRelativeToRobot(Rotation2d angle) {
    hubAngleRelativeToRobot = angle;
  } // End setHubAngleRelativeToRobot

  /** Get the current hub angle. */
  public Rotation2d getHubAngleRelativeToRobot() {
    return hubAngleRelativeToRobot;
  } // End getHubAngleRelativeToRobot

  /** Get the current Turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads + kEncoderZeroOffsetRad);
  } // End getPosition

  /** Whether the requested hub angle is within turret physical limits (not clamped). */
  public boolean isHubInRange() {
    double hubRad = hubAngleRelativeToRobot.getRadians();
    return hubRad >= kMinAngleRad && hubRad <= kMaxAngleRad;
  } // End isHubInRange

  /** Whether the Turret is at the hub within tolerance. */
  public boolean aimedAtHub() {
    double hubRadClamped = getClampedHubAngleRad();
    double currentAngleRad = turretInputs.positionRads + kEncoderZeroOffsetRad;
    return Math.abs(MathUtil.angleModulus(currentAngleRad - hubRadClamped)) <= kAtHubToleranceRad;
  } // End aimedAtHub

  /** Hub angle (robot frame) clamped to turret min/max, in radians. */
  private double getClampedHubAngleRad() {
    return MathUtil.clamp(hubAngleRelativeToRobot.getRadians(), kMinAngleRad, kMaxAngleRad);
  } // End getClampedHubAngleRad
}

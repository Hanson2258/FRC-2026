package frc.robot.subsystems.shooter.turret;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kD;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kDefaultTurretRads;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kEncoderZeroOffsetRad;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kI;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxAngleRad;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMinAngleRad;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kP;

/** Turret subsystem: one motor with onboard position control, aimed at a hub angle. */
public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIO.TurretIOInputs turretInputs = new TurretIO.TurretIOInputs();

  private static final double kAtHubToleranceRad = Units.degreesToRadians(2.0);
  private Rotation2d hubAngleRelativeToRobot = Rotation2d.kZero;
  private double velocityFeedforwardRadPerSec = 0.0;

  /** When false, the turret will automatically aim towards the hub */
  private BooleanSupplier manualOverrideSupplier = () -> false;
  /** When true (and not manual override), turret aims at shot target; when false, holds current position. */
  private BooleanSupplier aimAtTargetSupplier = () -> false;
  private Drive drive;

  private double lastSmartDashboardTargetPos = 0;

  public Turret(TurretIO io) {
    turretIO = io;

    SmartDashboard.putNumber("Turret/kP", kP);
    SmartDashboard.putNumber("Turret/kI", kI);
    SmartDashboard.putNumber("Turret/kD", kD);
    SmartDashboard.putNumber("Turret/TargetPositionRads", turretInputs.targetPositionRads);
  } // End Turret Constructor

  /** Set by RobotContainer so calculator does not overwrite Turret when operator is in manual override. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

  /** Set by RobotContainer so Turret can get Robot Pose. */
  public void setDrive(Drive drive) {
    this.drive = drive;
  } // End setDrive

  /** Set by RobotContainer so turret only aims at target when e.g. ShootWhenReadyCommand is active. */
  public void setAimAtTargetSupplier(BooleanSupplier supplier) {
    aimAtTargetSupplier = supplier != null ? supplier : () -> true;
  } // End setAimAtTargetSupplier

  @Override
  public void periodic() {
    double targetPositionRad = DriverStation.isDisabled() ? 0.0 : getClampedHubAngleRad() - kEncoderZeroOffsetRad;

    // When not in manual override, aim at the hub only if aim-at-target is enabled (e.g. ShootWhenReadyCommand active); otherwise hold position
    if (!manualOverrideSupplier.getAsBoolean()) {
      if (aimAtTargetSupplier.getAsBoolean()) {
        setHubAngleRelativeToRobot(ShooterCommands.getTurretAngleFromShot(drive));
        setVelocityFeedforwardRadPerSec(-drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
      } else {
        setVelocityFeedforwardRadPerSec(0.0);
      }
    } else {
      double target = SmartDashboard.getNumber("Turret/TargetPositionRads", kDefaultTurretRads);
      if (target != lastSmartDashboardTargetPos) {
        setHubAngleRelativeToRobot(new Rotation2d(target));
      }

      lastSmartDashboardTargetPos = target;
    }

    turretIO.updateInputs(turretInputs);
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

  /** Resets the motors position to 0 */
  public void resetMotorEncoder() {
    turretIO.stop();
    turretIO.resetEncoder();
    setHubAngleRelativeToRobot(new Rotation2d(0));
    SmartDashboard.putNumber("Turret/TargetPositionRads", 0);
  } // End resetMotorEncoder

  /** Set the hub angle (robot frame: 0 = forward). Clamped to min/max in periodic. */
  public void setHubAngleRelativeToRobot(Rotation2d angle) {
    hubAngleRelativeToRobot = angle;
  } // End setHubAngleRelativeToRobot

  /** Step the target Rads by the given amount. */
  public void stepRads(double stepRads) {
    setHubAngleRelativeToRobot(new Rotation2d(turretInputs.targetPositionRads).plus(new Rotation2d(stepRads)));
  } // End stepRads

  /** Get the current hub angle. */
  public Rotation2d getHubAngleRelativeToRobot() {
    return hubAngleRelativeToRobot;
  } // End getHubAngleRelativeToRobot

  /** Get the current Turret position (robot frame: 0 = forward). */
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretInputs.positionRads + kEncoderZeroOffsetRad);
  } // End getPosition

  /** Get the current target position (robot frame: 0 = forward). */
  public Rotation2d getTargetPosition() {
    return Rotation2d.fromRadians(turretInputs.targetPositionRads + kEncoderZeroOffsetRad);
  } // End getTargetPosition

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

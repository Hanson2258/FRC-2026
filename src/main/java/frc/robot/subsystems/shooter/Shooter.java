package frc.robot.subsystems.shooter;

import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.shooter.transfer.Transfer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Coordinates Agitator, Transfer, Turret, Hood, Flywheel; updates targets from lookup; exposes ready-to-shoot. */
public class Shooter extends SubsystemBase {

  private final Drive drive;
  @SuppressWarnings("unused")
  private final Agitator agitator;
  @SuppressWarnings("unused")
  private final Transfer transfer;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final boolean hoodEnabled;

  private BooleanSupplier shootCommandScheduledSupplier = () -> false;
  /** Set by ShootWhenReadyCommand in initialize/end so active is true when running. */
  private volatile boolean shootCommandActive = false;

  public Shooter(
      Drive drive,
      Agitator agitator,
      Transfer transfer,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      boolean hoodEnabled) {
    this.drive = drive;
    this.agitator = agitator;
    this.transfer = transfer;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.hoodEnabled = hoodEnabled;
  } // End Shooter Constructor

  /** Set by RobotContainer so Shooter can log whether shoot-when-ready is active. */
  public void setShootCommandScheduledSupplier(BooleanSupplier supplier) {
    shootCommandScheduledSupplier = supplier != null ? supplier : () -> false;
  } // End setShootCommandScheduledSupplier

  /** Set by ShootWhenReadyCommand in initialize/end so active is true whenever the command is running. */
  public void setShootCommandActive(boolean active) {
    shootCommandActive = active;
  } // End setShootCommandActive

  /** True when ShootWhenReadyCommand is running. */
  public boolean isShootCommandActive() {
    return shootCommandScheduledSupplier.getAsBoolean() || shootCommandActive;
  } // End isShootCommandActive

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterCommand/Target", ShooterCommands.getShooterTargetName());
    ShooterCommands.setShooterTarget(drive, turret, hood, flywheel, hoodEnabled);
    Logger.recordOutput("ShooterCommand/ShootWhenReadyCommandActive", shootCommandScheduledSupplier.getAsBoolean() || shootCommandActive);
    Logger.recordOutput("ShooterCommand/Ready/IsReadyToShoot", isReadyToShoot());
    Logger.recordOutput("ShooterCommand/Ready/AllianceZoneOk", !ShooterCommands.isShooterTargetHub() || AllianceUtil.isInAllianceZone(drive.getPose().getX()));
    Logger.recordOutput("ShooterCommand/Ready/TurretHubInRange", turret.isHubInRange());
    Logger.recordOutput("ShooterCommand/Ready/TurretAtTarget", turret.aimedAtHub());
    Logger.recordOutput("ShooterCommand/Ready/HoodAtTarget", !hoodEnabled || hood.atTarget());
    Logger.recordOutput("ShooterCommand/Ready/FlywheelAtTarget", flywheel.atTargetVelocity());
    Logger.recordOutput("ShooterCommand/Ready/FlywheelNotIdle", flywheel.getState() != FlywheelState.IDLE);
  } // End periodic

  /** Turret aimed at hub (hub in range and at target), Flywheel not idle and at speed; (Optional) Hood at target. When target is hub, robot must be in alliance zone. */
  public boolean isReadyToShoot() {
    if (ShooterCommands.isShooterTargetHub() && !AllianceUtil.isInAllianceZone(drive.getPose().getX())) {
      return false;
    }
    if (!turret.isHubInRange()) return false;
    if (!turret.aimedAtHub()) return false;
    if (flywheel.getState() == FlywheelState.IDLE) return false;
    if (!flywheel.atTargetVelocity()) return false;
    if (hoodEnabled && !hood.atTarget()) return false;
    return true;
  } // End isReadyToShoot
}

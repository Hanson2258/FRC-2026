package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.State;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.transfer.Transfer;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.AllianceUtil;

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
  /** When true, ShooterCommands.setShooterTarget will not apply calculator to Hood/Flywheel (Manual Override). */
  private BooleanSupplier manualOverrideSupplier = () -> false;

  /** Will automatically select the shooting target (Hub, left/right passing zones) when true, when false it will target the hub */
  public boolean autoSelectShootingTarget = true;

  /** Accumulated time flywheel has been off target velocity; reset when back on target. */
  private double flywheelOffTargetGraceTimerSec = 0.0;
  private double flywheelGracePrevTimestampSec = Double.NaN;

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

  /** Set by RobotContainer so calculator does not overwrite Hood/Flywheel when operator is in manual override. */
  public void setManualOverrideSupplier(BooleanSupplier supplier) {
    manualOverrideSupplier = supplier != null ? supplier : () -> false;
  } // End setManualOverrideSupplier

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
    double nowSec = Timer.getFPGATimestamp();
    double dtSec =
        Double.isNaN(flywheelGracePrevTimestampSec) ? 0.0 : (nowSec - flywheelGracePrevTimestampSec);
    flywheelGracePrevTimestampSec = nowSec;
    if (flywheel.atTargetVelocity()) {
      flywheelOffTargetGraceTimerSec = 0.0;
    } else {
      flywheelOffTargetGraceTimerSec += dtSec;
    }

    Logger.recordOutput("ShooterCommand/Target", ShooterCommands.getShooterTargetName());
    Logger.recordOutput("ShooterCommand/ShootWhenReadyCommandActive", shootCommandScheduledSupplier.getAsBoolean() || shootCommandActive);
    Logger.recordOutput("ShooterCommand/Ready/IsReadyToShoot", isReadyToShoot());
    Logger.recordOutput("ShooterCommand/Ready/AllianceZoneOk", !ShooterCommands.isShooterTargetHub() || AllianceUtil.isInAllianceZone(drive.getPose().getX()));
    Logger.recordOutput("ShooterCommand/Ready/HubMinDistanceOk", hubAutoshootDistanceOk());
    Logger.recordOutput("ShooterCommand/Ready/TurretTargetInRange", turret.isTargetInRange());
    Logger.recordOutput("ShooterCommand/Ready/TurretAtTarget", turret.atTarget());
    Logger.recordOutput("ShooterCommand/Ready/HoodAtTarget", !hoodEnabled || hood.atTarget());
    Logger.recordOutput("ShooterCommand/Ready/FlywheelAtTarget", flywheel.atTargetVelocity());
    Logger.recordOutput("ShooterCommand/Ready/FlywheelAtTargetWithinGracePeriod", !(flywheelOffTargetGraceTimerSec >= ShooterConstants.kFlywheelOffTargetGraceSec));
    Logger.recordOutput("ShooterCommand/Ready/FlywheelNotIdle", flywheel.getState() != State.IDLE);

    ShooterCommands.setShooterTarget(drive, turret, hood, flywheel, hoodEnabled, !manualOverrideSupplier.getAsBoolean());
  } // End periodic

  /**
   * Turret target in range and on target, Flywheel not Idle and at target speed (with {@link
   * ShooterConstants#kFlywheelOffTargetGraceSec} grace after leaving target speed); (Optional) Hood at target. When
   * shooter target is hub, robot must be in alliance zone and at least {@link
   * ShooterConstants#kMinHubAutoshootDistanceM} from hub center.
   */
  public boolean isReadyToShoot() {
    if (ShooterCommands.isShooterTargetHub()) {
      if (!AllianceUtil.isInAllianceZone(drive.getPose().getX())) {
        return false;
      }
      if (!hubAutoshootDistanceOk()) {
        return false;
      }
    }
    if (!turret.isTargetInRange()) return false;
    if (!turret.atTarget()) return false;
    if (flywheel.getState() == State.IDLE) return false;
    if (flywheelOffTargetGraceTimerSec >= ShooterConstants.kFlywheelOffTargetGraceSec) return false;
    if (hoodEnabled && !hood.atTarget()) return false;
    return true;
  } // End isReadyToShoot

  /** True if not shooting at hub, or robot is far enough from the alliance hub for autoshoot. */
  private boolean hubAutoshootDistanceOk() {
    if (!ShooterCommands.isShooterTargetHub()) {
      return true;
    }
    Translation2d hubCenter =
        AllianceUtil.isRedAlliance() ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;
    return drive.getPose().getTranslation().getDistance(hubCenter) >= ShooterConstants.kMinHubAutoshootDistanceM;
  } // End hubAutoshootDistanceOk
}

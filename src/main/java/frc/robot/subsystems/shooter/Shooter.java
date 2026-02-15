package frc.robot.subsystems.shooter;

import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.shooter.transfer.Transfer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  @Override
  public void periodic() {
    ShooterCommands.setShooterTarget(drive, hood, flywheel);
  } // End periodic

  /** Turret aimed, Flywheel not idle and at speed; (Optional) Hood at target. */
  public boolean isReadyToShoot() {
    if (!turret.aimedAtHub()) return false;
    if (flywheel.getState() == FlywheelState.IDLE) return false;
    if (!flywheel.atTargetVelocity()) return false;
    if (hoodEnabled && !hood.atTarget()) return false;
    return true;
  } // End isReadyToShoot
}

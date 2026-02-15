package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.shooter.transfer.Transfer;

/**
 * While scheduled: when Shooter is ready (Turret aimed, Flywheel at speed, (Optional) Hood at target),
 * runs Transfer then 0.25 s later Agitator (shooting mode). Idles both on end/cancel.
 */
public class ShootWhenReadyCommand extends Command {

  private static final double kAgitatorDelaySec = 0.25;
  private static final double kLoopPeriodSec = 0.02;

  private final Agitator agitator;
  private final Transfer transfer;
  private final Shooter shooter;

  private boolean transferOn = false;
  private double timerSec = 0.0;

  public ShootWhenReadyCommand(Agitator agitator, Transfer transfer, Shooter shooter) {
    this.agitator = agitator;
    this.transfer = transfer;
    this.shooter = shooter;
    addRequirements(agitator, transfer, shooter);
  } // End ShootWhenReadyCommand Constructor

  @Override
  public void initialize() {
    transferOn = false;
    timerSec = 0.0;
    shooter.setShootCommandActive(true);
  } // End initialize

  @Override
  public void execute() {
    if (!shooter.isReadyToShoot()) return;

    if (!transferOn) {
      transfer.setShootingMode();
      transferOn = true;
      timerSec = 0.0;
    }

    timerSec += kLoopPeriodSec;
    if (timerSec >= kAgitatorDelaySec) {
      agitator.setShootingMode();
    }
  } // End execute

  @Override
  public void end(boolean interrupted) {
    shooter.setShootCommandActive(false);
    transfer.setIdleMode();
    agitator.setIdleMode();
  } // End end

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled (toggle off)
  } // End isFinished
}

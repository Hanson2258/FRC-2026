package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

/**
 * While scheduled: when Shooter is ready (Turret aimed, Flywheel at speed, (Optional) Hood at target),
 * runs Transfer then 0.25 s later Agitator (shooting mode). Idles both on end/cancel.
 */
public class ShooterTargetCommand extends Command {

  private final Supplier<Pose2d> position;

  public ShooterTargetCommand(Supplier<Pose2d> position) {
    this.position = position;
  } // End ShootWhenReadyCommand Constructor

  @Override
  public void initialize() {
  } // End initialize

  @Override
  public void execute() {
    // Aim at hub if in alliance zone, otherwise depending on the side of the field pass balls back into the alliance zone
    if (position.get().getX() < FieldConstants.ALLIANCE_ZONE_M) {
     ShooterCommands.clearShooterTargetOverride();
    }
    else  {
     if (position.get().getY() > FieldConstants.FIELD_CENTER_Y_M) {
    	  ShooterCommands.setPassingSpotLeft();
     } else {
    	  ShooterCommands.setPassingSpotRight();
     }
    }

    //System.out.println("Current passing spot: " + ShooterCommands.getShooterTargetName());
  } // End execute

  @Override
  public void end(boolean interrupted) {
    ShooterCommands.clearShooterTargetOverride();
  } // End end

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled (toggle off)
  } // End isFinished
}

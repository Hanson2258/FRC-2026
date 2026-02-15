package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;

/** Shooter-related command helpers (hood, flywheel, turret aim via ShooterCalculator). */
public final class ShooterCommands {

  private ShooterCommands() {}

  /**
   * Alliance hub center in 3D (meters) for current alliance.
   */
  public static Translation3d getHubCenter3d() {
    return AllianceUtil.isRedAlliance() ? FieldConstants.RED_HUB_CENTER_3D : FieldConstants.BLUE_HUB_CENTER_3D;
  }

  /**
   * Field-frame angle from turret pivot to alliance hub (direction to aim in field).
   */
  public static Rotation2d getFieldAngleToHubFromPivot(Drive drive) {
    return ShooterCalculator.calculateAzimuthAngleFieldFrame(drive.getPose(), getHubCenter3d());
  }

  /**
   * Turret angle in robot frame (0 = robot forward) to aim at alliance hub from pivot.
   */
  public static Rotation2d getTurretAngleToHubFromPivot(Drive drive) {
    return ShooterCalculator.calculateAzimuthAngleRobotFrame(drive.getPose(), getHubCenter3d());
  }

  /**
   * Sets hood and flywheel target from ShooterCalculator (funnel clearance + moving shot). Applies
   * phase delay, then iterative moving shot; clamps hood to mechanism limits.
   */
  public static void setShooterTarget(Drive drive, Hood hood, Flywheel flywheel) {
    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeChassisSpeeds();
    Translation3d hub3d = getHubCenter3d();

    // Phase delay: predict pose forward so shot is for when ball actually leaves
    double dt = ShooterConstants.kPhaseDelaySec;
    Pose2d estimatedPose =
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(
                        fieldSpeeds.vxMetersPerSecond * dt, fieldSpeeds.vyMetersPerSecond * dt)),
            pose.getRotation().plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * dt)));

    ShotData shot =
        ShooterCalculator.iterativeMovingShotFromFunnelClearance(
            estimatedPose, fieldSpeeds, hub3d, ShooterConstants.kLookaheadIterations);

    double distanceM = ShooterCalculator.getDistanceToTarget(estimatedPose, shot.target());
    Logger.recordOutput("Shooter/DistanceToHubMeters", distanceM);
    Logger.recordOutput(
        "Shooter/CalculatorAngleDegrees", Units.radiansToDegrees(shot.hoodAngleFromVerticalRad()));
    double flywheelRadsPerSec =
        ShooterCalculator.linearToAngularVelocity(
            shot.exitVelocityMps(), FlywheelConstants.kFlywheelRadiusMeters);
    Logger.recordOutput("Shooter/CalculatorVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(flywheelRadsPerSec));
    Logger.recordOutput("Shooter/ExitVelocityMps", shot.exitVelocityMps());

    double hoodRad =
        MathUtil.clamp(
            shot.hoodAngleFromVerticalRad(),
            HoodConstants.kMinAngleRad,
            HoodConstants.kMaxAngleRad);
    hood.setTargetAngleRad(hoodRad);
    flywheel.setTargetVelocityRadsPerSec(flywheelRadsPerSec);
  } // End setShooterTarget
}

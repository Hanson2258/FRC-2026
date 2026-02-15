package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
   * Alliance hub center (top of Funnel) in 3D (meters) for current alliance.
   */
  public static Translation3d getShooterTarget3d() {
    return AllianceUtil.isRedAlliance()
        ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D
        : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;
  }

  /**
   * Field-frame angle from turret pivot to alliance hub (direction to aim in field).
   */
  public static Rotation2d getFieldAngleToHubFromPivot(Drive drive) {
    Pose2d pose = drive.getPose();
    return pose.getRotation().plus(
        Rotation2d.fromRadians(
            ShooterCalculator.calculateAzimuthAngle(pose, getShooterTarget3d()).in(Radians)));
  }

  /**
   * Turret angle in robot frame (0 = robot forward) to aim at alliance hub from pivot.
   */
  public static Rotation2d getTurretAngleToHubFromPivot(Drive drive) {
    return Rotation2d.fromRadians(
        ShooterCalculator.calculateAzimuthAngle(drive.getPose(), getShooterTarget3d()).in(Radians));
  }

  /**
   * Sets hood and flywheel target from ShooterCalculator. When hoodEnabled is false, uses a fixed
   * hood angle and solves for velocity only so the shot matches the locked hood. When true, uses
   * funnel clearance + moving shot. Applies phase delay, then iterative moving shot; clamps hood to
   * mechanism limits.
   */
  public static void setShooterTarget(
      Drive drive, Hood hood, Flywheel flywheel, boolean hoodEnabled) {
    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeChassisSpeeds();
    Translation3d target3d = getShooterTarget3d();

    // Phase delay: predict pose forward so shot is for when ball actually leaves
    double dt = ShooterConstants.kPhaseDelaySec;
    Pose2d estimatedPose =
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(
                        fieldSpeeds.vxMetersPerSecond * dt, fieldSpeeds.vyMetersPerSecond * dt)),
            pose.getRotation().plus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * dt)));

    ShotData shot;
    if (hoodEnabled) {
      shot =
          ShooterCalculator.iterativeMovingShotFromFunnelClearance(
              estimatedPose, fieldSpeeds, target3d, ShooterConstants.kLookaheadIterations);
    } else {
      shot =
          ShooterCalculator.iterativeMovingShotWithFixedHoodAngle(
              estimatedPose,
              fieldSpeeds,
              target3d,
              ShooterConstants.kFixedHoodAngleWhenDisabledRad,
              ShooterConstants.kLookaheadIterations);
    }

    double distanceM =
        ShooterCalculator.getDistanceToTarget(estimatedPose, shot.getTarget()).in(Meters);
    Logger.recordOutput("Shooter/DistanceToHubMeters", distanceM);
    Logger.recordOutput(
        "Shooter/CalculatorAngleDegrees", Units.radiansToDegrees(shot.getHoodAngle().in(Radians)));
    double exitVelMps =
        shot.getExitVelocity().in(MetersPerSecond) * ShooterConstants.kExitVelocityCompensationMultiplier;
    double flywheelRadsPerSec =
        ShooterCalculator.linearToAngularVelocity(
                MetersPerSecond.of(exitVelMps), Meters.of(FlywheelConstants.kFlywheelRadiusMeters))
            .in(RadiansPerSecond);
    Logger.recordOutput("Shooter/CalculatorVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(flywheelRadsPerSec));
    Logger.recordOutput("Shooter/ExitVelocityMps", exitVelMps);

    double hoodRad =
        MathUtil.clamp(
            shot.getHoodAngle().in(Radians),
            HoodConstants.kMinAngleRad,
            HoodConstants.kMaxAngleRad);
    hood.setTargetAngleRad(hoodRad);
    flywheel.setTargetVelocityRadsPerSec(flywheelRadsPerSec);
  } // End setShooterTarget
}

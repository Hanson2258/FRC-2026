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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.AllianceUtil;
import org.littletonrobotics.junction.Logger;

/** Shooter-related command helpers (Hood, Flywheel, Turret aim via ShooterCalculator). */
public final class ShooterCommands {

  private ShooterCommands() {}

  private static final String kTargetAimOffsetDegKey = "Shooter/TargetAimOffsetDeg";
  private static final String kExitVelocityMultiplierAdditiveKey =
      "Shooter/ExitVelocityCompensationMultiplierAdditive";
  private static final double kDefaultTargetAimOffsetDeg = 0.0;
  private static final double kDefaultExitVelocityMultiplierAdditive = 0.0;

  static {
    SmartDashboard.putNumber(kTargetAimOffsetDegKey, kDefaultTargetAimOffsetDeg);
    SmartDashboard.putNumber(kExitVelocityMultiplierAdditiveKey, kDefaultExitVelocityMultiplierAdditive);
  }

  /** Which passing spot is selected; null = aim at hub. */
  private static volatile PassingSpot passingSpotOverride = null;

  /** Turret angle to predicted target from last setShooterTarget (accounts for robot velocity). */
  private static volatile Rotation2d lastTurretAngleFromShot = null;

  public enum PassingSpot {
    LEFT,
    CENTER,
    RIGHT
  }

  /** Set target to passing spot left (alliance-relative). */
  public static void setPassingSpotLeft() {
    passingSpotOverride = PassingSpot.LEFT;
  }

  /** Set target to passing spot center (alliance-relative). */
  public static void setPassingSpotCenter() {
    passingSpotOverride = PassingSpot.CENTER;
  }

  /** Set target to passing spot right (alliance-relative). */
  public static void setPassingSpotRight() {
    passingSpotOverride = PassingSpot.RIGHT;
  }

  /** Clear override so Shooter returns to hub. */
  public static void clearShooterTargetOverride() {
    passingSpotOverride = null;
  }

  /** True if current target is the hub (no passing-spot override). */
  public static boolean isShooterTargetHub() {
    return passingSpotOverride == null;
  }

  private static Translation3d getPassingSpot3d(PassingSpot spot) {
    boolean red = AllianceUtil.isRedAlliance();
    return switch (spot) {
      case LEFT -> red ? FieldConstants.RED_PASSING_SPOT_LEFT : FieldConstants.BLUE_PASSING_SPOT_LEFT;
      case CENTER -> red ? FieldConstants.RED_PASSING_SPOT_CENTER : FieldConstants.BLUE_PASSING_SPOT_CENTER;
      case RIGHT -> red ? FieldConstants.RED_PASSING_SPOT_RIGHT : FieldConstants.BLUE_PASSING_SPOT_RIGHT;
    };
  }

  /** Current Shooter target: passing spot or alliance hub (funnel top). */
  public static Translation3d getShooterTarget3d() {
    PassingSpot spot = passingSpotOverride;
    if (spot != null) return getPassingSpot3d(spot);
    return AllianceUtil.isRedAlliance()
        ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D
        : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;
  }

  /** Get current target for logging. */
  public static String getShooterTargetName() {
    PassingSpot spot = passingSpotOverride;
    if (spot == null) return "Hub";
    return switch (spot) {
      case LEFT -> "Passing Left";
      case CENTER -> "Passing Center";
      case RIGHT -> "Passing Right";
    };
  }

  /**
   * Field-frame angle from Turret pivot to alliance hub (direction to aim in field).
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
   * Turret angle from last shot solution (aim at predicted target so ball + robot velocity hits hub).
   * Fallback to hub angle if shot not yet computed this cycle.
   */
  public static Rotation2d getTurretAngleFromShot(Drive drive) {
    if (lastTurretAngleFromShot != null) return lastTurretAngleFromShot;
    return getTurretAngleToHubFromPivot(drive);
  }

  /**
   * Sets Hood and Flywheel target from ShooterCalculator. When hoodEnabled is false, uses a fixed
   * Hood angle and solves for velocity only so the shot matches the locked Hood. When true, uses
   * funnel clearance + moving shot. Applies phase delay, then iterative moving shot; clamps Hood to
   * mechanism limits. Turret aim uses predicted target and shortest-path azimuth.
   * When {@code enableCalculator} is false (e.g. manual override), Hood and Flywheel targets are
   * not updated so manual override controls are functional.
   */
  public static void setShooterTarget(Drive drive, Turret turret, Hood hood, Flywheel flywheel, boolean hoodEnabled, boolean enableCalculator) {
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

    double distanceM = ShooterCalculator.getDistanceToTarget(estimatedPose, shot.getTarget()).in(Meters);
    Logger.recordOutput("Shooter/DistanceToHubMeters", distanceM);
    Logger.recordOutput("Shooter/CalculatorHoodDeg", Units.radiansToDegrees(shot.getHoodAngle().in(Radians)));
    double exitVelMps = shot.getExitVelocity().in(MetersPerSecond);
    double exitVelocityMultiplierAdditive =
        SmartDashboard.getNumber(kExitVelocityMultiplierAdditiveKey, kDefaultExitVelocityMultiplierAdditive);
    double flywheelSurfaceSpeedMps = exitVelMps / ShooterConstants.kFlywheelSurfaceDivider
            * (ShooterConstants.kExitVelocityCompensationMultiplier() + exitVelocityMultiplierAdditive);
    double flywheelRadPerSec = ShooterCalculator.linearToAngularVelocity(
            MetersPerSecond.of(flywheelSurfaceSpeedMps), Meters.of(FlywheelConstants.kFlywheelRadiusMeters)).in(RadiansPerSecond);
    Logger.recordOutput("Shooter/CalculatorVelocityRpm", Units.radiansPerSecondToRotationsPerMinute(flywheelRadPerSec));
    Logger.recordOutput("Shooter/ExitVelocityMps", exitVelMps);
    Logger.recordOutput(kExitVelocityMultiplierAdditiveKey, exitVelocityMultiplierAdditive);

    double hoodAngleRad =
        MathUtil.clamp(
            shot.getHoodAngle().in(Radians),
            HoodConstants.kMinAngleRad,
            HoodConstants.kMaxAngleRad);
    if (enableCalculator) {
      hood.setTargetAngleRad(hoodAngleRad);
      flywheel.setTargetVelocityRadPerSec(flywheelRadPerSec);
    }

    // We provide the turret "current angle" to the calculator in the turret's internal frame
    // so shortest-path selection respects kMinAngleRad/kMaxAngleRad.
    double targetAimOffsetDegAdditive = SmartDashboard.getNumber(kTargetAimOffsetDegKey, kDefaultTargetAimOffsetDeg);
    lastTurretAngleFromShot = Rotation2d.fromRadians(
          ShooterCalculator.calculateAzimuthAngle(
                estimatedPose, shot.getTarget(), turret.getPosition().getRadians())
            .in(Radians)).plus(Rotation2d.fromDegrees(ShooterConstants.kTargetAimOffsetDeg + targetAimOffsetDegAdditive));
    Logger.recordOutput(kTargetAimOffsetDegKey, ShooterConstants.kTargetAimOffsetDeg + targetAimOffsetDegAdditive);
  } // End setShooterTarget
}

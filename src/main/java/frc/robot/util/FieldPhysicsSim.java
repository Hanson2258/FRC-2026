package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Physics simulation for field boundaries and obstacles. Prevents the robot from driving through
 * walls and field structures.
 */
public class FieldPhysicsSim {
  private final Drive drive;
  private final double fieldLength;
  private final double fieldWidth;
  private final double robotRadius; // Approximate robot radius for collision detection

  /**
   * Creates a new FieldPhysicsSim.
   *
   * @param drive The drive subsystem to monitor and constrain
   * @param robotRadius The approximate radius of the robot in meters (used for collision detection)
   */
  public FieldPhysicsSim(Drive drive, double robotRadius) {
    this.drive = drive;
    this.robotRadius = robotRadius;

    // Get field dimensions from AprilTag layout
    this.fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
    this.fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
  }

  /**
   * Updates the physics simulation. Should be called from Robot.simulationPeriodic(). Checks if
   * the robot is outside field boundaries and resets it to the nearest valid position if so.
   */
  public void update() {
    Pose2d currentPose = drive.getPose();
    Translation2d position = currentPose.getTranslation();

    // Check if robot is outside field boundaries (with robot radius buffer)
    double minX = robotRadius;
    double maxX = fieldLength - robotRadius;
    double minY = robotRadius;
    double maxY = fieldWidth - robotRadius;

    double clampedX = MathUtil.clamp(position.getX(), minX, maxX);
    double clampedY = MathUtil.clamp(position.getY(), minY, maxY);

    // If position was clamped, reset the robot pose to the clamped position
    if (clampedX != position.getX() || clampedY != position.getY()) {
      Pose2d clampedPose = new Pose2d(clampedX, clampedY, currentPose.getRotation());
      drive.setPose(clampedPose);
    }
  }
}

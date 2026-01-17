package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.InputStream;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem that uses PhotonVision to estimate robot pose from AprilTags.
 * This subsystem provides pose estimates that can be fused with odometry in the drivetrain.
 */
public class Vision extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout;
  private double lastEstimateTimestamp = 0;
  private VisionSim visionSim;
  private edu.wpi.first.math.geometry.Transform3d robotToCameraTransform;

  /**
   * Creates a new Vision subsystem.
   * 
   * @param cameraName The name of the PhotonVision camera (must match the camera name in PhotonVision UI)
   */
  public Vision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    
    // Get the robot-to-camera transform (used for both real and simulation)
    robotToCameraTransform = getRobotToCameraTransform();
    
    // Load the AprilTag field layout
    // Note: WPILib 2026 doesn't include 2026 field layouts in AprilTagFields enum yet,
    // so we load the custom 2026 field layout JSON from resources
    // File location: src/main/resources/2026-rebuilt-welded.json
    try {
      fieldLayout = new AprilTagFieldLayout("src/main/resources/2026-rebuilt-welded.json");
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTag field layout: " + e.getMessage(), e.getStackTrace());
      fieldLayout = null;
    }

    // Create pose estimator (no longer takes PhotonCamera in 2026)
    // Strategy is now specified when calling estimate methods, not in constructor
    if (fieldLayout != null) {
      poseEstimator = new PhotonPoseEstimator(
          fieldLayout,
          robotToCameraTransform
      );
      
      // Initialize vision simulation if in simulation mode
      visionSim = new VisionSim(camera, fieldLayout, robotToCameraTransform);
      
      // Add simulation field visualization to SmartDashboard
      Field2d simField = visionSim.getSimDebugField();
      if (simField != null) {
        SmartDashboard.putData("Vision Sim Field", simField);
      }
    } else {
      poseEstimator = null;
    }
  }

  /**
   * Get the transform from the robot center to the camera.
   * This should be configured based on your camera mounting position.
   * 
   * @return Transform3d from robot center to camera
   */
  private edu.wpi.first.math.geometry.Transform3d getRobotToCameraTransform() {
    // TODO: Configure these values based on your camera mounting position
    // X: forward, Y: left, Z: up (all in meters)
    // Rotation: camera orientation relative to robot
    return new edu.wpi.first.math.geometry.Transform3d(
        new edu.wpi.first.math.geometry.Translation3d(0.0, 0.0, 0.0), // Camera position relative to robot center
        new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, 0.0)    // Camera rotation (pitch, yaw, roll)
    );
  }

  @Override
  public void periodic() {
    // The pose estimation is done on-demand via getEstimatedGlobalPose()
    // This periodic method can be used for logging or other periodic tasks
  }

  /**
   * Update the vision simulation with the current robot pose.
   * This should be called from Robot.simulationPeriodic().
   * 
   * @param robotSimPose The current simulated robot pose
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    if (visionSim != null) {
      visionSim.simulationPeriodic(robotSimPose);
    }
  }

  /**
   * Reset the robot pose in the vision simulation.
   * 
   * @param pose The new robot pose
   */
  public void resetSimPose(Pose2d pose) {
    if (visionSim != null) {
      visionSim.resetSimPose(pose);
    }
  }

  /**
   * Get the latest estimated robot pose from vision.
   * 
   * @param prevEstimatedRobotPose The previous estimated robot pose (used for ambiguity resolution)
   * @return Optional containing the estimated pose and timestamp, or empty if no valid estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (poseEstimator == null || fieldLayout == null) {
      return Optional.empty();
    }

    // Process all unread results from the camera
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      // Try multi-tag estimation first (most accurate)
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      
      // Fallback to single tag if multi-tag fails
      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      
      // Update timestamp tracking
      double latestTimestamp = result.getTimestampSeconds();
      boolean hasNewResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;
      
      if (hasNewResult) {
        lastEstimateTimestamp = latestTimestamp;
      }
    }
    
    return visionEst;
  }

  /**
   * Get the latest camera result for debugging or other purposes.
   * Note: getLatestResult() is deprecated in 2026. This method gets the most recent unread result.
   * 
   * @return The latest PhotonPipelineResult from the camera, or null if no results
   */
  public PhotonPipelineResult getLatestResult() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return null;
    }
    // Return the most recent result (last in the list)
    return results.get(results.size() - 1);
  }

  /**
   * Check if the camera has a valid target.
   * 
   * @return true if a target is detected, false otherwise
   */
  public boolean hasTargets() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return false;
    }
    // Check the most recent result
    return results.get(results.size() - 1).hasTargets();
  }
}

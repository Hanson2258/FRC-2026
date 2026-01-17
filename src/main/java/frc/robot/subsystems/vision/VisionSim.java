package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Vision simulation support for PhotonVision. This class handles simulating the camera and vision
 * system in simulation mode.
 */
public class VisionSim {
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;
  private final AprilTagFieldLayout fieldLayout;

  /**
   * Creates a new VisionSim instance.
   *
   * @param camera The PhotonCamera to simulate
   * @param fieldLayout The AprilTag field layout
   * @param robotToCameraTransform The transform from robot center to camera
   */
  public VisionSim(
      PhotonCamera camera,
      AprilTagFieldLayout fieldLayout,
      edu.wpi.first.math.geometry.Transform3d robotToCameraTransform) {
    this.fieldLayout = fieldLayout;

    // Only initialize simulation if we're in simulation mode
    if (RobotBase.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field
      visionSim = new VisionSystemSim("main");

      // Add all the AprilTags from the field layout as visible targets
      visionSim.addAprilTags(fieldLayout);

      // Create simulated camera properties
      // These can be tuned to mimic your actual camera characteristics
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(70);
      cameraProp.setAvgLatencyMs(30);
      cameraProp.setLatencyStdDevMs(10);

      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible targets from the simulation
      cameraSim = new PhotonCameraSim(camera, cameraProp);

      // Add the simulated camera to view the targets on this simulated field
      visionSim.addCamera(cameraSim, robotToCameraTransform);

      // Enable wireframe visualization (optional, for debugging)
      cameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * Update the vision simulation with the current robot pose. This should be called periodically
   * during simulation.
   *
   * @param robotSimPose The current simulated robot pose
   */
  public void simulationPeriodic(Pose2d robotSimPose) {
    if (RobotBase.isSimulation() && visionSim != null) {
      visionSim.update(robotSimPose);
    }
  }

  /**
   * Reset the robot pose in the vision system simulation.
   *
   * @param pose The new robot pose
   */
  public void resetSimPose(Pose2d pose) {
    if (RobotBase.isSimulation() && visionSim != null) {
      visionSim.resetRobotPose(pose);
    }
  }

  /**
   * Get a Field2d for visualizing the robot and objects on the field in simulation. This can be
   * added to SmartDashboard for visualization.
   *
   * @return Field2d for visualization, or null if not in simulation
   */
  public Field2d getSimDebugField() {
    if (!RobotBase.isSimulation() || visionSim == null) {
      return null;
    }
    return visionSim.getDebugField();
  }
}

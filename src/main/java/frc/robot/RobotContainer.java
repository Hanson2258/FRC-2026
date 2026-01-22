// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Manual Override and Encoder Reset
  public static boolean manualOverride = false;
  private boolean encoderReset = false;

  // Face Target mode
  private boolean isFacingHub = false;
  private ProfiledPIDController faceTargetController;

  // Maple-sim simulation (only used in SIM mode)
  private SwerveDriveSimulation driveSimulation = null;


  /** 
   * RobotContainer constructor initializes the robot. 
   */
  public RobotContainer() {
    // Initialize subsystems based on mode (REAL, SIM, or REPLAY)
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // Initialize vision after drive (vision needs drive reference)
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        // Configure simulation timing for accurate physics
        // 5 ticks per 20ms period = 4kHz effective control loop rate
        SimulatedArena.overrideSimulationTimings(
            edu.wpi.first.units.Units.Seconds.of(0.02), // 20ms robot period
            5); // 5 simulation ticks per period
        
        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        // Initialize vision after drive (vision needs drive reference)
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
        break;

      // Replayed robot, disable IO implementations
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Initialize vision after drive (vision needs drive reference)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add SysId routines to auto chooser
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Initialize face-target PID controller (using same constants as DriveCommands)
    faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
        new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
    faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

    // Configure button bindings
    configureDriveBindings(true); // False to disable driving
    configureOperatorBindings(false); // False to disable operator controls
  }



  /** Prints the current odometry pose of the robot to the console. */
  public void printPose() {
    Pose2d robotPose = drive.getPose();
    System.out.println("=== Odometry Pose ===");
    System.out.println("X:   " + String.format("%.3f", robotPose.getX()) + " m");
    System.out.println("Y:   " + String.format("%.3f", robotPose.getY()) + " m");
    System.out.println("Rot: " + String.format("%.2f", robotPose.getRotation().getDegrees()) + " deg");
    System.out.println("====================");
  } // End printPose


  /**
   * Configure only the drive to enable or disable
   *
   * @param enableDriving true to enable driving, false to disable
   */
  private void configureDriveBindings(boolean enableDriving) {
    // Can't configure if drive is null
    if (drive == null) {return;}

    // Drive disabled: stop all movement
    if (!enableDriving) {      
      drive.setDefaultCommand(
          Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), drive));
      return;
    }

    // Drive enabled: field-relative drive with turbo control and optional face-target mode
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithTurboAndFaceTarget(
            drive,
            () -> -driverController.getLeftX(), // X-axis (left/right)
            () -> -driverController.getLeftY(), // Y-axis (forward/backward)
            () -> -driverController.getRightX(), // Omega (rotation)
            () -> driverController.getRightTriggerAxis(), // Turbo
            () -> isFacingHub, // Face-target enabled
            faceTargetController,
            false)); // usePhysicalMaxSpeed: false = use artificial limit (1.6 m/s), true = use physical max

    // Reset gyro / odometry - syncs with simulation pose in SIM mode
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(
            driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
        : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)); // Zero gyro on real robot
    driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(() -> drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),drive).ignoringDisable(true));

    // Toggle face-target mode when Y button is pressed
    driverController.y().onTrue(Commands.runOnce(() -> {
      isFacingHub = !isFacingHub;
      if (isFacingHub) {
        // Reset PID controller when enabling face-target mode
        faceTargetController.reset(drive.getRotation().getRadians());
      }
    }, drive));

    driverController.a().onTrue(Commands.runOnce(() -> printPose()));

    // Pathfind then follow path to outpost when D-pad up is held
    driverController.povUp().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "DriveToOutpost"));
  }

  /** 
   * Configure operator controls
   */
  private void configureOperatorBindings(boolean enableOperatorControls) {
    // Operator Controls Enabled
    if (enableOperatorControls){
      // Add operator controls here
    }
  } // End configureOperatorBindings


  /**
   * Run the path selected from the auto chooser
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Reset the simulation field for autonomous mode. Only works in SIM mode.
   */
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /**
   * Update the simulation world. Should be called from Robot.simulationPeriodic().
   * Only works in SIM mode.
   */
  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}

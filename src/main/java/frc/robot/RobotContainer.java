// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Swerve Drive constants
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts top speed possible at 12 volts
  private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second, max angular velocity
  private final double MAX_CONTROL_SPEED = 1.6; // Max speed the driver can go in x or y in m/s
  private final double TURBO_MULTIPLE = 2; // Technically a divider for how slow it is pre-turbo since it is limited to the max control speed

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private static final double XBOX_DEADBAND = 0.09;

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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // Initialize vision after drive (vision needs drive reference)
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
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

  /**
   * Deadband function to eliminate small joystick inputs.
   * @param value to apply deadband to.
   */
  private static double applyDeadband(double value) {
		return applyDeadband(value, XBOX_DEADBAND);
	} // End applyDeadband

  /** 
   * Deadband function to eliminate small joystick inputs.
   * @param value to apply deadband to.
   * @param deadband threshold.
   */
  private static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    // Rescale so the output goes from 0 to 1 outside the deadband
    double sign = Math.signum(value);
    double adjusted = (Math.abs(value) - deadband) / (1.0 - deadband);
    return sign * adjusted;
  } // End applyDeadband

  /**
   * Scale a raw joystick axis to a control output, applying a deadband and a "turbo" multiplier.
   *
   * Behavior:
   * - Applies the existing deadband to `inputAxis` (expected in range [-1, 1]).
   * - Interpolates `turboAxis` (expected in [0, 1]) linearly between a minimum turbo
   *   value (1 / TURBO_MULTIPLE) and 1.0, then scales the result.
   *
   * @param inputAxis  Raw joystick axis in [-1.0, 1.0]. Positive/negative direction is preserved.
   * @param turboAxis  Turbo level in [0.0, 1.0] (0 = minimum speed limiter; 1 = full speed). Values outside that range will be clamped.
   * @param maxRange   Maximum magnitude of the output (units chosen by caller; e.g. meters/sec or radians/sec). Should be >= 0.
   */
  private double scaleAxisWithTurbo(double inputAxis, double turboAxis, double maxRange) {
    inputAxis = applyDeadband(inputAxis); // Apply the joystick deadband

    // Linear interpolation from minTurbo (when turboAxis == 0) to 1.0 (when turboAxis == 1).
    double minTurbo = 1.0 / TURBO_MULTIPLE; // Minimum fraction of maxRange when turbo is not applied
    double turbo = turboAxis * (1.0 - minTurbo) + minTurbo;

    // Scale the (signed) axis by the physical range and the turbo multiplier
    return inputAxis * maxRange * turbo;
  } // End scaleAxisWithTurbo


  /**
   * Converts robot-relative speeds to field-relative speeds, accounting for alliance color.
   *
   * @param robotRelativeSpeeds Speeds relative to the robot's current orientation
   * @return Speeds relative to the field, flipped if on red alliance
   */
  private ChassisSpeeds convertToFieldRelative(ChassisSpeeds robotRelativeSpeeds) {
    boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Rotation2d fieldOrientation = isRedAlliance ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    return ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, fieldOrientation);
  }

  /**
   * Calculates the target angle to face the alliance's hub center.
   *
   * @return The desired rotation to face the target hub
   */
  private Rotation2d calculateTargetHubAngle() {
    // Get robot's current position
    Pose2d robotPose = drive.getPose();
    Translation2d robotPosition = robotPose.getTranslation();

    // Determine target based on alliance
    boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d targetPosition = isRedAlliance ? FieldConstants.RED_HUB_CENTER : FieldConstants.BLUE_HUB_CENTER;

    // Calculate angle from robot to target
    Translation2d delta = targetPosition.minus(robotPosition);
    return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
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
    drive.setDefaultCommand(Commands.run(() -> {
      // Read joystick inputs
      double leftY = -driverController.getLeftY(); // Forward/backward
      double leftX = -driverController.getLeftX(); // Left/right
      double rightX = -driverController.getRightX(); // Rotation
      double turboAxis = driverController.getRightTriggerAxis(); // Turbo multiplier

      // Calculate velocities with turbo and deadband
      double velocityX = scaleAxisWithTurbo(leftY, turboAxis, MAX_CONTROL_SPEED);
      double velocityY = scaleAxisWithTurbo(leftX, turboAxis, MAX_CONTROL_SPEED);
      
      // Determine rotational rate: use face-target PID if enabled, otherwise use joystick
      double rotationalRate;
      if (isFacingHub) {
        // Calculate target angle and use PID controller to rotate toward it
        Rotation2d targetAngle = calculateTargetHubAngle();
        rotationalRate = faceTargetController.calculate(drive.getRotation().getRadians(), targetAngle.getRadians());
      } else {
        // Use joystick input for rotation
        rotationalRate = scaleAxisWithTurbo(rightX, turboAxis, MAX_ANGULAR_RATE);
        // Reset PID controller when not using face-target mode
        faceTargetController.reset(drive.getRotation().getRadians());
      }

      // Convert to field-relative speeds
      ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(velocityX, velocityY, rotationalRate);
      ChassisSpeeds fieldRelativeSpeeds = convertToFieldRelative(robotRelativeSpeeds);

      // Execute drive command
      drive.runVelocity(fieldRelativeSpeeds);
    }, drive));

    // Reset the field-centric heading on Start button press
    driverController.start().onTrue(
        Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)), drive));

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
}

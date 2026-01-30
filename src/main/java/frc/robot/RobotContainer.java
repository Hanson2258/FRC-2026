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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turrent.Turret;
import frc.robot.subsystems.shooter.turrent.TurretIO;
import frc.robot.subsystems.shooter.turrent.TurretIOSim;
import frc.robot.subsystems.shooter.turrent.TurretIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// Controller(s)
	private final CommandXboxController driverController = new CommandXboxController(0);

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Vision vision;
	private final Turret turret; 

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;

	// Field view (robot + turret pose) – visible in SmartDashboard/Glass when running sim
	private final Field2d field = new Field2d();
	private final FieldObject2d turretObject = field.getObject("Turret");

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

  // Manual Override and Encoder Reset
  public static boolean manualOverride = false;
  @SuppressWarnings("unused")
  private boolean encoderReset = false;

  // Face Target mode
  private boolean isFacingHub = false;
  private ProfiledPIDController faceTargetController;


	/** The container for the robot. Contains subsystems, OI devices, and commands. */
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
								new ModuleIOTalonFX(TunerConstants.BackRight),
								(pose) -> {});
				// Initialize vision after drive (vision needs drive reference)
				this.vision =
					new Vision(
						drive,
						new VisionIOPhotonVision(camera0Name, robotToCamera0),
						new VisionIOPhotonVision(camera1Name, robotToCamera1));
        this.turret = new Turret(new TurretIOSparkMax());
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
				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(
								TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
						new ModuleIOSim(
								TunerConstants.FrontRight, driveSimulation.getModules()[1]),
						new ModuleIOSim(
								TunerConstants.BackLeft, driveSimulation.getModules()[2]),
						new ModuleIOSim(
								TunerConstants.BackRight, driveSimulation.getModules()[3]),
						driveSimulation::setSimulationWorldPose);
				// Initialize vision after drive (vision needs drive reference)
				this.vision =
						new Vision(
								drive,
								new VisionIOPhotonVisionSim(
										camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
								new VisionIOPhotonVisionSim(
										camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
				this.turret = new Turret(new TurretIOSim());
				break;

			// Replayed robot, disable IO implementations
			default:
				drive = new Drive(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						(pose) -> {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
				turret = new Turret(new TurretIO() {});

				break;
		}

		// Turret aims at hub using robot pose (odometry); replace with hold-position command to disable
		turret.setDefaultCommand(
				Commands.run(
						() -> turret.setGoalAngle(DriveCommands.getTurretAngleToHub(drive)),
						turret));

		// Field view: robot + turret so you can see turret direction in sim
		SmartDashboard.putData("Field", field);

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
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

	// Switch to X pattern when X button is pressed
	driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

	// Reset gyro / odometry
	final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
			? () -> drive.setPose(
					driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
			// simulation
			: () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
	driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

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

    // Pathfind then follow path to hub when D-pad down is held
    driverController.povDown().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "DriveToHub"));
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
	 * Use this to pass the autonomous command to the main {@link Robot} class.
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
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);

		// Turret pose in field frame
		double robotX = robotPose.getX();
		double robotY = robotPose.getY();
		double robotTheta = robotPose.getRotation().getRadians();
		double dx = ShooterConstants.robotToTurret.getX();
		double dy = ShooterConstants.robotToTurret.getY();
		double turretX = robotX + dx * Math.cos(robotTheta) - dy * Math.sin(robotTheta);
		double turretY = robotY + dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);
		Rotation2d turretAngle = robotPose.getRotation().plus(turret.getPosition());
		Pose2d turretPose = new Pose2d(turretX, turretY, turretAngle);
		Logger.recordOutput("FieldSimulation/TurretPose", turretPose);

		// 3D turret pose (Z from robotToTurret) so it shows in the air in 3D view like camera poses
		double turretZ = ShooterConstants.robotToTurret.getZ();
		Pose3d turretPose3d =
				new Pose3d(turretX, turretY, turretZ, new Rotation3d(0, 0, turretAngle.getRadians()));
		Logger.recordOutput("FieldSimulation/TurretPose3d", turretPose3d);

		// Update field view so you can see robot and turret direction in the dashboard
		field.setRobotPose(robotPose);
		turretObject.setPose(turretPose);
	}
}

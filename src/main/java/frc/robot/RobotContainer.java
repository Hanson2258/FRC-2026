// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.agitator.AgitatorConstants;
import frc.robot.subsystems.agitator.AgitatorIO;
import frc.robot.subsystems.agitator.AgitatorIOSim;
import frc.robot.subsystems.agitator.AgitatorIOSparkMax;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.shooter.ShooterSimVisualizer;
import frc.robot.subsystems.shooter.transfer.Transfer;
import frc.robot.subsystems.shooter.transfer.TransferConstants;
import frc.robot.subsystems.shooter.transfer.TransferIO;
import frc.robot.subsystems.shooter.transfer.TransferIOBrushedSparkMax;
import frc.robot.subsystems.shooter.transfer.TransferIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.simulation.FuelSim;
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

	// Controller
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);

	// Subsystems Toggle
	private boolean isDriveEnabled = true;
	private boolean isVisionEnabled = true;
	private boolean isIntakeEnabled = true;
	private boolean isAgitatorEnabled = true;
	private boolean isTransferEnabled = true;
	private boolean isTurretEnabled = true;
	private boolean isHoodEnabled = false;
	private boolean isFlywheelEnabled = true;

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Vision vision;
	private final Intake intake;
	private final Agitator agitator;
	private final Transfer transfer;
	private final Turret turret;
	private final Hood hood;
	private final Flywheel flywheel;

	// Shooter Manag
	private final Shooter shooter;
	private final ShootWhenReadyCommand shootWhenReadyCommand;

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;

	// Fuel simulation (robot-ball collision in sim)
	private final FuelSim fuelSim = new FuelSim();

	// Shooter sim and visualizer (only non-null in SIM)
	private final ShooterSim shooterSim;
	private final ShooterSimVisualizer shooterSimVisualizer;

	// Field view (robot pose)
	private final Field2d field = new Field2d();

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

  // Manual Override
  public static boolean manualOverride = true; // TODO: Implement manual override properly, and change back to false

  // Face Target mode
  private boolean isFacingHub = false;
  private ProfiledPIDController faceTargetController;

  // Robot-centric vs field-centric drive
  private boolean isRobotCentric = false;


	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
    // Initialize subsystems based on mode (REAL, SIM, or REPLAY)
		switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
			case REAL:
				if (isDriveEnabled) {
					drive =
							new Drive(
									new GyroIOPigeon2(),
									new ModuleIOTalonFX(TunerConstants.FrontLeft),
									new ModuleIOTalonFX(TunerConstants.FrontRight),
									new ModuleIOTalonFX(TunerConstants.BackLeft),
									new ModuleIOTalonFX(TunerConstants.BackRight),
									(pose) -> {});
				} else {
					drive = new Drive(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						(pose) -> {});
				}
				
				// Initialize vision after drive (vision needs drive reference)
				if (isVisionEnabled) {
					vision =
							new Vision(
									drive,
									new VisionIOPhotonVision(camera0Name, robotToCamera0),
									new VisionIOPhotonVision(camera1Name, robotToCamera1));
				} else {
					vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
				}

				// Subsystems
				intake = isIntakeEnabled ? new Intake(new IntakeIOSparkMax()) : new Intake(new IntakeIO() {});
				agitator = isAgitatorEnabled ? new Agitator(new AgitatorIOSparkMax()) : new Agitator(new AgitatorIO() {});
				transfer = isTransferEnabled ? new Transfer(new TransferIOBrushedSparkMax()) : new Transfer(new TransferIO() {});
				turret = isTurretEnabled ? new Turret(new TurretIOSparkMax()) : new Turret(new TurretIO() {});
				hood = isHoodEnabled ? new Hood(new HoodIOSparkMax()) : new Hood(new HoodIO() {});
				flywheel = isFlywheelEnabled ? new Flywheel(new FlywheelIOTalonFX()) : new Flywheel(new FlywheelIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
        // Configure simulation timing for accurate physics
        // 5 ticks per 20ms period = 4kHz effective control loop rate
        SimulatedArena.overrideSimulationTimings(
            edu.wpi.first.units.Units.Seconds.of(0.02), // 20ms robot period
            5); // 5 simulation ticks per period

        // Use local arena with to drive through ramps
        SimulatedArena.overrideInstance(new frc.robot.simulation.Arena2026Rebuilt(true));

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
				vision =
						new Vision(
								drive,
								new VisionIOPhotonVisionSim(
										camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
								new VisionIOPhotonVisionSim(
										camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

				// Subsystems
				intake = new Intake(new IntakeIOSim());
				agitator = new Agitator(new AgitatorIOSim());
				transfer = new Transfer(new TransferIOSim());
				turret = new Turret(new TurretIOSim());
				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());

				shooterSim = new ShooterSim(fuelSim);
				shooterSimVisualizer =
						new ShooterSimVisualizer(
								() ->
										new Pose3d(
												drive.getPose().getX(),
												drive.getPose().getY(),
												0,
												new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())),
								drive::getFieldRelativeChassisSpeeds);

				configureFuelSim();
				configureFuelSimRobot(true, shooterSim::intakeFuel);
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

				// Subsystems
				intake = new Intake(new IntakeIO() {});
				agitator = new Agitator(new AgitatorIO() {});
				transfer = new Transfer(new TransferIO() {});
				turret = new Turret(new TurretIO() {});
				hood = new Hood(new HoodIO() {});
				flywheel = new Flywheel(new FlywheelIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;
			}

		// Shooter coordinator and shoot-when-ready command (Option B)
		shooter = new Shooter(drive, agitator, transfer, turret, hood, flywheel, isHoodEnabled);
		shootWhenReadyCommand = new ShootWhenReadyCommand(agitator, transfer, shooter);
		shooter.setShootCommandScheduledSupplier(shootWhenReadyCommand::isScheduled);

		/// ---------------------------------------------------------------------------------------------------------------
		/// ----------------------------------------------- Drive Commands ------------------------------------------------
		/// ---------------------------------------------------------------------------------------------------------------		// Initialize face-target PID controller (using same constants as DriveCommands)
		faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
		new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
    faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		/// ---------------------------------------------------------------------------------------------------------------
		/// ------------------------------------------ Shooter Subsystem Commands -----------------------------------------
		/// ---------------------------------------------------------------------------------------------------------------
		// Turret aims at hub using robot pose (odometry); replace with hold-position command to disable
		turret.setDefaultCommand(
				Commands.run(
						() -> turret.setHubAngleRelativeToRobot(ShooterCommands.getTurretAngleToHubFromPivot(drive)),
						turret));

		/// ---------------------------------------------------------------------------------------------------------------
		/// ----------------------------------------------- Logger Dashboard ----------------------------------------------
		/// ---------------------------------------------------------------------------------------------------------------
		// Field view: robot + turret so you can see turret direction in sim
		SmartDashboard.putData("Field", field);

		// Register PathPlanner named commands
		registerCommands();

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


		// Record zeroed robot components (model_0 turret, model_1 extender) – initial only; updated in updateSimulation()
		Logger.recordOutput("ComponentPoses/Zeroed", new Pose3d[] {new Pose3d(), new Pose3d()});
		Logger.recordOutput("ComponentPoses/Final",
				new Pose3d[] {
					new Pose3d(-0.17, 0.05, 0.35, new Rotation3d(0, 0, 0)), // model_0 turret
					new Pose3d(0.5, 0, 0.35, new Rotation3d(0, 0, 0))  // model_1 extender
				});
			

    // Configure button bindings
    configureDriverBindings(true); // False to disable driving
    configureOperatorBindings(true); // False to disable operator controls
  }


  /**
   * Configures FuelSim for robot-ball collision in simulation.
   */
  private void configureFuelSim() {
    fuelSim.setShowHalfFuel(false);
		fuelSim.enableAirResistance();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
		SmartDashboard.putData(Commands.runOnce(() -> {
						fuelSim.clearFuel();
						fuelSim.spawnStartingFuel();
				})
				.withName("Reset Fuel")
				.ignoringDisable(true));
  }
	
	/** Configures the robot for fuel simulation. */ // TODO: change ableToIntake to BooleanSupplier when Extender is implemented
	private void configureFuelSimRobot(boolean ableToIntake, Runnable intakeCallback) {
    // Robot Sizing
    double robotWidthMeters =
        TunerConstants.FrontLeft.LocationY - TunerConstants.FrontRight.LocationY;
    double robotLengthMeters =
        TunerConstants.FrontLeft.LocationX - TunerConstants.BackLeft.LocationX;
    double bumperHeightMeters = 0.35;

		// Register a robot for collision with fuel
    fuelSim.registerRobot(
        robotWidthMeters,
        robotLengthMeters,
        bumperHeightMeters,
        drive::getPose,
        drive::getFieldRelativeChassisSpeeds);

		// Register intakes for the robot
		// Intake: 10.5" beyond front of frame (+X), full width minus 2" on each side
		double intakeExtendMeters = 10.5 * 0.0254;
		double intakeInsetMeters = 2.0 * 0.0254;
    fuelSim.registerIntake(
				robotLengthMeters / 2,
				robotLengthMeters / 2 + intakeExtendMeters,
				-robotWidthMeters / 2 + intakeInsetMeters,
				robotWidthMeters / 2 - intakeInsetMeters,
				// () -> intake.isRightDeployed() && ableToIntake, // TODO: Uncomment and fix when Extender is implemented
				intakeCallback);
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
  private void configureDriverBindings(boolean enableDriving) {
    // Drive disabled: stop all movement
    if (!enableDriving) {      
      drive.setDefaultCommand(
          Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), drive));
      return;
    }
		else {
			// Drive enabled: field/robot-centric drive with turbo and optional face-target
			drive.setDefaultCommand(
					DriveCommands.joystickDriveWithTurboAndFaceTarget(
							drive,
							() -> -driverController.getLeftX(), // X-axis (left/right)
							() -> -driverController.getLeftY(), // Y-axis (forward/backward)
							() -> -driverController.getRightX(), // Omega (rotation)
							() -> driverController.getRightTriggerAxis(), // Turbo
							() -> isFacingHub, // Face-target enabled
							() -> isRobotCentric, // Robot-centric (true) vs field-centric (false)
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

			// Toggle robot-centric vs field-centric drive
			driverController.rightBumper().onTrue(Commands.runOnce(() -> isRobotCentric = !isRobotCentric, drive));


			// -------- Auto Pathfind to Target --------
			// Pathfind then follow path to outpost when D-pad up is held
			driverController.leftStick().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "DriveToOutpost"));
			// Pathfind then follow path to hub when D-pad down is held
			driverController.rightStick().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "DriveToHub"));
		} // End else (Drive enabled)

    // Shoot toggle: on = schedule ShootWhenReadyCommand, set Flywheel to Charging if IDLE; off = cancel (command end() idles Transfer and Agitator)
		driverController.a().onTrue(Commands.runOnce(() -> {
			if (shootWhenReadyCommand.isScheduled()) {
				CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
			} else {
				if (flywheel != null && flywheel.getState() == FlywheelState.IDLE) {
					flywheel.setState(FlywheelState.CHARGING);
				}
				CommandScheduler.getInstance().schedule(shootWhenReadyCommand);
			}
		}));

    // POV up = back to hub; Pass to our side: POV left/right = passing spot; POV down = center
		driverController.povUp().onTrue(Commands.runOnce(ShooterCommands::clearShooterTargetOverride));
    driverController.povLeft().onTrue(Commands.runOnce(ShooterCommands::setPassingSpotLeft));
    driverController.povRight().onTrue(Commands.runOnce(ShooterCommands::setPassingSpotRight));
    driverController.povDown().onTrue(Commands.runOnce(ShooterCommands::setPassingSpotCenter));

    // Flywheel: Toggles Idle ↔ Charging/AtSpeed (Charging auto-transitions to AtSpeed when at target)
    if (flywheel != null) {
      driverController.leftBumper().onTrue(
          Commands.runOnce(
              () -> {
                if (flywheel.getState() == FlywheelState.IDLE) {
                  flywheel.setState(FlywheelState.CHARGING);
                } else {
                  flywheel.setState(FlywheelState.IDLE);
                }
              },
              flywheel));
    }

		// -------- Manual Override + Encoder Reset --------
		// If Manual Override is false, become true
		// If Manual Override is true, reset encoder positions, and then become false
		driverController.back().onTrue(Commands.runOnce(() -> 
			new ConditionalCommand(
				new ParallelCommandGroup(
					// TODO: Reset encoder positions
					Commands.runOnce(() -> manualOverride = false)
				), 
				Commands.runOnce(() -> manualOverride = true),
				() -> manualOverride)
			));
  }

  /** 
   * Configure operator controls
   */
  private void configureOperatorBindings(boolean enableOperatorControls) {
    // Operator Controls Enabled
    if (enableOperatorControls) {
			// Enable/ Disable Intake
			operatorController.leftTrigger().onTrue(Commands.runOnce(() -> intake.setIntakingMode(), intake));
			operatorController.leftTrigger().onFalse(Commands.runOnce(() -> intake.setIdleMode(), intake));
			operatorController.rightTrigger().onTrue(Commands.runOnce(() -> intake.setReversingMode(), intake));
			operatorController.rightTrigger().onFalse(Commands.runOnce(() -> intake.setIdleMode(), intake));

			// Set Agitator, Transfer, and Flywheel to idle mode when B is pressed
			operatorController.b().onTrue(Commands.runOnce(() -> {
				if (agitator != null) agitator.setIdleMode();
				if (transfer != null) transfer.setIdleMode();
				if (flywheel != null) flywheel.setState(FlywheelState.IDLE);
			}, agitator, transfer, flywheel));

			// Manual Override for Intake Voltage
			if (manualOverride && intake != null) {
				final double stepVoltage = 0.25; // TODO: Set step voltage
				operatorController.povLeft().onTrue(
						Commands.runOnce(
								() -> {
									double next = Math.min(IntakeConstants.kMaxVoltage, intake.getTargetVoltage() + stepVoltage);
									if (intake.getMode() == Intake.Mode.IDLE) {
										intake.setIntakingMode();
										intake.setTargetVoltage(stepVoltage);
									} else {
										intake.setTargetVoltage(next);
									}
									if (next == 0) {
										intake.setIdleMode();
									}
								},
								intake));
				operatorController.povRight().onTrue(
						Commands.runOnce(
								() -> {
									double next = Math.max(-IntakeConstants.kMaxVoltage, intake.getTargetVoltage() - stepVoltage);
									if (intake.getMode() == Intake.Mode.IDLE) {
										intake.setReversingMode();
										intake.setTargetVoltage(-stepVoltage);
									} else {
										intake.setTargetVoltage(next);
									}
									if (next == 0) {
										intake.setIdleMode();
									}
								},
								intake));
			}

			// Manual Override for Agitator Voltage
			if (manualOverride && agitator != null) {
				final double stepVoltage = 0.25; // TODO: Set step voltage
				operatorController.y().onTrue(
						Commands.runOnce(
								() -> {
									double next = Math.min(AgitatorConstants.kMaxVoltage, agitator.getTargetVoltage() + stepVoltage);
									if (agitator.getMode() == Agitator.Mode.IDLE) {
										agitator.setStagingMode();
										agitator.setTargetVoltage(stepVoltage);
									} else {
										agitator.setTargetVoltage(next);
									}
									if (next == 0) {
										agitator.setIdleMode();
									}
								},
								agitator));
				operatorController.a().onTrue(
						Commands.runOnce(
								() -> {
									double next = Math.max(-AgitatorConstants.kMaxVoltage, agitator.getTargetVoltage() - stepVoltage);
									if (agitator.getMode() == Agitator.Mode.IDLE) {
										agitator.setStagingMode();
										agitator.setTargetVoltage(-stepVoltage);
									} else {
										agitator.setTargetVoltage(next);
									}
									if (next == 0) {
										agitator.setIdleMode();
									}
								},
								agitator));
			}

			// Manual Override for Transfer Voltage
			if (manualOverride && transfer != null) {
				final double stepVoltage = 0.25; // TODO: Set step voltage
				operatorController.leftBumper().onTrue(
						Commands.runOnce(
							() -> {
								double next = Math.min(TransferConstants.kMaxVoltage, transfer.getTargetVoltage() + stepVoltage);
								if (transfer.getMode() == Transfer.Mode.IDLE) {
									transfer.setStagingMode();
									transfer.setTargetVoltage(stepVoltage);
								} else {
									transfer.setTargetVoltage(next);
								}
								if (next == 0) {
									transfer.setIdleMode();
								}
							},
							transfer));
				operatorController.rightBumper().onTrue(
						Commands.runOnce(
							() -> {
								double next = Math.max(-TransferConstants.kMaxVoltage, transfer.getTargetVoltage() - stepVoltage);
								if (transfer.getMode() == Transfer.Mode.IDLE) {
									transfer.setStagingMode();
									transfer.setTargetVoltage(-stepVoltage);
								} else {
									transfer.setTargetVoltage(next);
								}
								if (next == 0) {
									transfer.setIdleMode();
								}
							},
							transfer));
			}

			// Manual Override for Flywheel Velocity
      if (manualOverride && flywheel != null) {
        final double stepRpm = 50.0; // TODO: Set step velocity
        final double stepRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(stepRpm);
        operatorController.povUp().onTrue(
            Commands.runOnce(
                () -> {
                  double current = flywheel.getTargetVelocityRadsPerSec();
                  double next = current + stepRadsPerSec;
                  flywheel.setTargetVelocityRadsPerSec(next);
                  flywheel.setState(FlywheelState.CHARGING);
                },
                flywheel));
        operatorController.povDown().onTrue(
            Commands.runOnce(
                () -> {
                  double current = flywheel.getTargetVelocityRadsPerSec();
                  double next = Math.max(0, current - stepRadsPerSec);
                  flywheel.setTargetVelocityRadsPerSec(next);
                  flywheel.setState(next == 0 ? FlywheelState.IDLE : FlywheelState.CHARGING);
                },
                flywheel));
      }

			// -------- Manual Override + Encoder Reset --------
			// If Manual Override is false, become true
			// If Manual Override is true, reset encoder positions, and then become false
			driverController.back().onTrue(Commands.runOnce(() -> 
				new ConditionalCommand(
					new ParallelCommandGroup(
						// TODO: Reset encoder positions
						Commands.runOnce(() -> manualOverride = false)
					), 
					Commands.runOnce(() -> manualOverride = true),
					() -> manualOverride)
				));
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
   * Update Field2d with the current robot pose. Call from Robot.robotPeriodic().
   * In SIM mode the pose is set in updateSimulation(); on real robot we use drive odometry.
   */
	public void updateFieldPose() {
		if (Constants.currentMode == Constants.Mode.SIM) return;
		field.setRobotPose(drive.getPose());
	}

  /**
   * Update the simulation world. Should be called from Robot.simulationPeriodic().
   * Only works in SIM mode.
   */
	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);

		// Robot-relative component poses for visualization
		Pose3d turretComponentPose = new Pose3d(-0.17, 0.05, 0.35, new Rotation3d(0, 0, turret.getPosition().getRadians() - Math.PI / 2.0));
		Pose3d extenderComponentPose = new Pose3d(0.5, 0, 0.35, new Rotation3d(0, 0, 0));
		Logger.recordOutput("ComponentPoses/Final", new Pose3d[] {turretComponentPose, extenderComponentPose});

		// Update field view
		field.setRobotPose(robotPose);

		// Shooter sim: launch fuel only when shooter is ready and shoot command is active
		if (shooterSim != null) {
			shooterSim.update(shooter, shooter::isShootCommandActive, turret, hood, flywheel);
		}
		if (shooterSimVisualizer != null) {
			double hoodAngleRad = isHoodEnabled ? hood.getAngleRad() : HoodConstants.kMinAngleRad;
			shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(
							flywheel.getTargetVelocityRadsPerSec()
									* FlywheelConstants.kFlywheelRadiusMeters),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad),
					edu.wpi.first.units.Units.Radians.of(turret.getPosition().getRadians()));
			shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(turret.getPosition().getRadians()),
					edu.wpi.first.units.Units.Radians.of(hoodAngleRad));
		}

		// Fuel sim (robot-ball collision)
		fuelSim.updateSim();

		// Log balls in robot and hub scores (FuelSim)
		if (shooterSim != null) {
			Logger.recordOutput("FuelSim/BallsInRobot", shooterSim.getFuelStored());
		}
		Logger.recordOutput("FuelSim/BlueHubScore", FuelSim.Hub.BLUE_HUB.getScore());
		Logger.recordOutput("FuelSim/RedHubScore", FuelSim.Hub.RED_HUB.getScore());
	}

	private void registerCommands() {
		// Register the commands here
		// Intake Commands
		NamedCommands.registerCommand("Intake On", Commands.runOnce(() -> intake.setIntakingMode(), intake));
		NamedCommands.registerCommand("Intake Off", Commands.runOnce(() -> intake.setIdleMode(), intake));
		NamedCommands.registerCommand("Intake Reverse", Commands.runOnce(() -> intake.setReversingMode(), intake));

		// Flywheel Commands
		NamedCommands.registerCommand("Flywheel On", Commands.runOnce(() -> flywheel.setState(FlywheelState.CHARGING), flywheel));
		NamedCommands.registerCommand("Flywheel Off", Commands.runOnce(() -> flywheel.setState(FlywheelState.IDLE), flywheel));

		// Shooter Target Commands
		NamedCommands.registerCommand("Set Shooter Target Hub", Commands.runOnce(ShooterCommands::clearShooterTargetOverride));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Left", Commands.runOnce(ShooterCommands::setPassingSpotLeft));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Center", Commands.runOnce(ShooterCommands::setPassingSpotCenter));
		NamedCommands.registerCommand("Set Shooter Target Passing Spot Right", Commands.runOnce(ShooterCommands::setPassingSpotRight));
		// With timeout so the sequential auto can advance to path commands (reference codebases build autos in code with paths only)
		NamedCommands.registerCommand("Shoot When Ready", shootWhenReadyCommand);
	}

	public void makeSystemSafe() {
		CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
		intake.setIdleMode();
		agitator.setIdleMode();
		transfer.setIdleMode();
		flywheel.setState(FlywheelState.IDLE);
	}
}
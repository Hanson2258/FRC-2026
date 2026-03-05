// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootWhenReadyCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import static frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.agitator.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.transfer.*;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelState;


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

	// Competition Toggle
	@AutoLogOutput(key = "CompetitionToggle")
	private boolean isCompetition = false;

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

	// Drive Commands
	private final TeleopDrive teleopDrive;

	// Robot Auto-Face Hub Target Mode
	@AutoLogOutput(key = "TeleopDrive/IsFacingHub")
	private boolean isFacingHub = false;
	private ProfiledPIDController faceTargetController;

	// Shooter Manager
	private final Shooter shooter;
	private final ShootWhenReadyCommand shootWhenReadyCommand;

	// Field view (robot pose)
	private final Field2d field = new Field2d();

	// Robot-centric vs Field-centric drive
	@AutoLogOutput(key = "TeleopDrive/IsRobotCentric")
	private boolean isRobotCentric = false;

	// Manual Override
	@AutoLogOutput(key = "ManualOverride/Driver")
	public static boolean driverManualOverride = false;
	@AutoLogOutput(key = "ManualOverride/Operator")
	public static boolean operatorManualOverride = false;

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	// Drive Simulation
	private SwerveDriveSimulation driveSimulation = null;

	// Fuel Simulation (robot-ball collision in SIM)
	private final FuelSim fuelSim = new FuelSim();

	// Shooter Simulation and Visualizer (only non-null in SIM)
	private final ShooterSim shooterSim;
	private final ShooterSimVisualizer shooterSimVisualizer;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
    // Initialize Subsystems based on mode (REAL, SIM, or REPLAY)
		switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
			case REAL:
				if (isDriveEnabled) {
					drive = new Drive(
							new GyroIOPigeon2(),
							new ModuleIOTalonFX(TunerConstants.FrontLeft), new ModuleIOTalonFX(TunerConstants.FrontRight),
							new ModuleIOTalonFX(TunerConstants.BackLeft), new ModuleIOTalonFX(TunerConstants.BackRight),
							(pose) -> {});
				} else {
					drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				}
				
				// Initialize Vision after drive (Vision needs Drive reference)
				if (isVisionEnabled) {
					vision = new Vision(drive,
							new VisionIOPhotonVision(camera0Name, robotToCamera0), 
							new VisionIOPhotonVision(camera1Name, robotToCamera1));
				} else {
					vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
				}

				// Subsystems
				intake   = isIntakeEnabled 	 ? new Intake(new IntakeIOSparkMax()) 					 : new Intake(new IntakeIO() {});
				agitator = isAgitatorEnabled ? new Agitator(new AgitatorIOSparkMax()) 			 : new Agitator(new AgitatorIO() {});
				transfer = isTransferEnabled ? new Transfer(new TransferIOSparkMax()) : new Transfer(new TransferIO() {});
				turret   = isTurretEnabled 	 ? new Turret(new TurretIOSparkMax()) 					 : new Turret(new TurretIO() {});
				hood     = isHoodEnabled  	 ? new Hood(new HoodIOSparkMax()) 							 : new Hood(new HoodIO() {});
				flywheel = isFlywheelEnabled ? new Flywheel(new FlywheelIOTalonFX()) 				 : new Flywheel(new FlywheelIO() {});
				shooterSim = null;
				shooterSimVisualizer = null;
				break;

			// Sim robot, instantiate physics sim IO implementations
			case SIM:
        // Configure Simulation timing for accurate physics
        // 5 ticks per 20ms period = 4kHz effective control loop rate
        SimulatedArena.overrideSimulationTimings(
            edu.wpi.first.units.Units.Seconds.of(0.02), // 20ms robot period
            5); // 5 Simulation ticks per period

        // Use local arena with to drive through ramps
        SimulatedArena.overrideInstance(new frc.robot.simulation.Arena2026Rebuilt(true));

				driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
						new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
						new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
						new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
						driveSimulation::setSimulationWorldPose);
				
				// Initialize Vision after drive (Vision needs Drive reference)
				vision = new Vision(drive,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

				// Subsystems
				intake = new Intake(new IntakeIOSim());
				agitator = new Agitator(new AgitatorIOSim());
				transfer = new Transfer(new TransferIOSim());
				turret = new Turret(new TurretIOSim());
				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());

				shooterSim = new ShooterSim(fuelSim);
				shooterSimVisualizer = new ShooterSimVisualizer(() -> {
							Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();
							return new Pose3d(
									simPose.getX(),
									simPose.getY(),
									0,
									new Rotation3d(0, 0, simPose.getRotation().getRadians()));
						},
						drive::getFieldRelativeChassisSpeeds);

				configureFuelSim();
				configureFuelSimRobot(true, shooterSim::intakeFuel);
				break;

			// Replayed Robot, disable IO implementations
			default:
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
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

		// Shooter coordinator and shoot-when-ready command
		shooter = new Shooter(drive, agitator, transfer, turret, hood, flywheel, isHoodEnabled);
		shootWhenReadyCommand = new ShootWhenReadyCommand(agitator, transfer, shooter);
		shooter.setShootCommandScheduledSupplier(shootWhenReadyCommand::isScheduled);
		shooter.setManualOverrideSupplier(() -> operatorManualOverride);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------- Drive Commands --------------------------------------		
		/// -------------------------------------------------------------------------------------------
		// Initialize face-target PID controller
		// Rotate so Turret pivot aims at Hub, not Robot center. Same PID constants as DriveCommands.
		faceTargetController = new ProfiledPIDController(DriveCommands.getAngleKp(), 0.0, DriveCommands.getAngleKd(),
		new TrapezoidProfile.Constraints(DriveCommands.getAngleMaxVelocity(), DriveCommands.getAngleMaxAcceleration()));
		faceTargetController.enableContinuousInput(-Math.PI, Math.PI);

		teleopDrive = new TeleopDrive(drive, driverController, () -> isRobotCentric, () -> isFacingHub, faceTargetController);
		teleopDrive.setManualOverrideSupplier(() -> driverManualOverride);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------- Shooter Subsystem Commands --------------------------------
		/// -------------------------------------------------------------------------------------------
		// Turret aims at predicted target; velocity feedforward for spin compensation. Only active if not in manualOverride
		turret.setManualOverrideSupplier(() -> operatorManualOverride);
		turret.setDrive(drive);

		/// -------------------------------------------------------------------------------------------
		/// ------------------------------------ Logger Dashboard -------------------------------------
		/// -------------------------------------------------------------------------------------------
		// Field view: Robot + Turret so you can see Turret direction in sim
		SmartDashboard.putData("Field", field);

		// Register PathPlanner named commands
		registerCommands();

		// Set up Auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Things to set up if not in competition
		if (!isCompetition) {
			// Set up Swerve Calibration Programs
			DriveCommands.swerveCalibration(autoChooser, drive);

			// Record zeroed Robot components (model_0 turret, model_1 extender, model_2 extending-storage) – initial only; updated in updateSimulation()
			Logger.recordOutput("ComponentPoses/Zeroed", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
		}

		// Record zeroed Robot components (model_0 turret, model_1 extender) – initial only; updated in updateSimulation()
		Logger.recordOutput("ComponentPoses/Final",
				new Pose3d[] {
					new Pose3d(-0.125, -0.17, 0.27, new Rotation3d(0, 0, 0)), // model_0 turret
					new Pose3d(0.28, 0, 0.15, new Rotation3d(0, 0, 0)),  // model_1 extender
				});
			

    // Configure button bindings
    configureDriverBindings();
    configureOperatorBindings(true); // False to disable operator controls
  }

	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Controller Input -----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
  /** Configure Driver controls. */
  private void configureDriverBindings() {
    drive.setDefaultCommand(teleopDrive);

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

		// ----------------------------------- Driver Manual Override + Encoder Reset -----------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		driverController.back().onTrue(
			new ConditionalCommand(
				new ParallelCommandGroup(
					// TODO: Reset encoder positions
					Commands.runOnce(() -> driverManualOverride = false)
				),
				Commands.runOnce(() -> driverManualOverride = true),
				() -> driverManualOverride));
  }

  /** Configure Operator controls. */
  private void configureOperatorBindings(boolean enableOperatorControls) {
    // Operator Controls Enabled
    if (!enableOperatorControls) {
			return;
		}

		// Enable/ Disable Intake
		operatorController.leftTrigger().onTrue(Commands.runOnce(() -> intake.setIntakingMode(), intake));
		operatorController.leftTrigger().onFalse(Commands.runOnce(() -> intake.setIdleMode(), intake));

		operatorController.rightTrigger().onTrue(Commands.runOnce(() -> intake.setReversingMode(), intake));
		operatorController.rightTrigger().onFalse(Commands.runOnce(() -> intake.setIdleMode(), intake));

		// ---------------------------------- Operator Manual Override + Encoder Reset ----------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		operatorController.back().onTrue(
			new ConditionalCommand(
				new ParallelCommandGroup(
					// TODO: Reset encoder positions
					Commands.runOnce(() -> operatorManualOverride = false)
				),
				Commands.runOnce(() -> operatorManualOverride = true),
				() -> operatorManualOverride));

		// Set Agitator, Transfer, and Flywheel to idle mode when B is pressed
		operatorController.b().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					if (agitator != null) agitator.setIdleMode();
					if (transfer != null) transfer.setIdleMode();
					if (flywheel != null) flywheel.setState(FlywheelState.IDLE);
				}, agitator, transfer, flywheel),
				new InstantCommand(),
				() -> operatorManualOverride));

		// Intake Manual Voltage Control
		final double intakeStepVoltage = 0.25;
		// Raise Intake voltage
		operatorController.povLeft().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(intakeStepVoltage), intake),
				new InstantCommand(),
				() -> (operatorManualOverride && intake != null)
			)
		);
		// Lower Intake voltage
		operatorController.povRight().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.stepVoltage(-intakeStepVoltage), intake),
				new InstantCommand(),
				() -> (operatorManualOverride && intake != null)
			)
		);

		// Agitator Manual Voltage Control
		final double agitatorStepVoltage = 0.25;
		// Raise Agitator voltage
		operatorController.y().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> agitator.stepVoltage(agitatorStepVoltage), agitator),
				new InstantCommand(),
				() -> (operatorManualOverride && agitator != null)
			)
		);
		// Lower Agitator voltage
		operatorController.a().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> agitator.stepVoltage(-agitatorStepVoltage), agitator),
				new InstantCommand(),
				() -> (operatorManualOverride && agitator != null)
			)
		);

		// Transfer Manual Voltage Control
		final double transferStepVoltage = 0.25;
		// Raise Transfer voltage
		operatorController.leftBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(transferStepVoltage), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);
		// Lower Transfer voltage
		operatorController.rightBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> transfer.stepVoltage(-transferStepVoltage), transfer),
				new InstantCommand(),
				() -> (operatorManualOverride && transfer != null)
			)
		);

		// Set Turret angle to 0 (straight ahead) relative to robot
		operatorController.x().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.setHubAngleRelativeToRobot(new Rotation2d(0.0)), turret), 
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);

		// Reset Turret Encoder
		operatorController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.resetMotorEncoder(), turret), 
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);
		
		// Turret Manual Position Control
		double turretStepPosition = Units.degreesToRadians(5);
		// Step turret position up
		operatorController.leftStick().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepRads(turretStepPosition), turret), 
				new InstantCommand(), 
				() -> (operatorManualOverride && turret != null)
			)
		);
		
		// Step turret position down
		operatorController.rightStick().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> turret.stepRads(-turretStepPosition), turret), 
				new InstantCommand(), 
				() -> (operatorManualOverride && turret != null)
			)
		);

		// Flywheel Manual Velocity Control
		final double stepRpm = 50.0;
		final double stepRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(stepRpm);
		// Raise Flywheel rpm
		operatorController.povUp().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> flywheel.stepVelocityRadsPerSec(stepRadsPerSec), flywheel),
				new InstantCommand(),
				() -> (operatorManualOverride && flywheel != null)
			)
		);

		// Lower Flywheel rpm
		operatorController.povDown().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> flywheel.stepVelocityRadsPerSec(-stepRadsPerSec), flywheel),
				new InstantCommand(),
				() -> (operatorManualOverride && flywheel != null)
			)
		);
  } // End configureOperatorBindings


	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------- Autonomous Commands Only --------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Register the commands here */
	private void registerCommands() {
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
	} // End registerCommands

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	} // End getAutonomousCommand

	/// -----------------------------------------------------------------------------------------------------------------
	/// --------------------------------------------- Other Useful Methods ----------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
	/** Idle the ball handling subsystems.  */
	public void idleBallHandling() {
		CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
		intake.setIdleMode();
		agitator.setIdleMode();
		transfer.setIdleMode();
		flywheel.setState(FlywheelState.IDLE);
	} // End idleBallHandling

  /**
   * Update Field2d with the current robot pose. Call from Robot.robotPeriodic().
   * In SIM mode the pose is set in updateSimulation(); on real robot we use drive odometry.
   */
	public void updateFieldPose() {
		if (Constants.currentMode == Constants.Mode.SIM) return;
		field.setRobotPose(drive.getPose());
	} // End updateFieldPose


	/// -----------------------------------------------------------------------------------------------------------------
	/// ------------------------------------------------ Simulation Only ------------------------------------------------
	/// -----------------------------------------------------------------------------------------------------------------
  /** Reset the simulation field for autonomous mode. Only works in SIM mode. */
	public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	} // End resetSimulationField

  /** Configures FuelSim for robot-ball collision in simulation. */
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
  } // End configureFuelSim
	
	/** Configures the robot for fuel simulation. */ // TODO: change ableToIntake to BooleanSupplier when Extender is implemented
	private void configureFuelSimRobot(boolean ableToIntake, Runnable intakeCallback) {
		double robotWidthMeters = Constants.Dimensions.FULL_WIDTH.in(Meters);
		double robotLengthMeters = Constants.Dimensions.FULL_LENGTH.in(Meters);
		double bumperHeightMeters = Constants.Dimensions.BUMPER_HEIGHT.in(Meters);

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
	} // End configureFuelSimRobot

  /** Update the Simulation world. Should be called from Robot.simulationPeriodic(). Only works in SIM mode. */
	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();

		// Robot pose for visualization
		Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition", robotPose);

		// Robot-relative component poses for visualization
		Pose3d turretComponentPose = new Pose3d(-0.125, -0.17, 0.27, new Rotation3d(0, 0, turret.getPosition().getRadians() + Math.toRadians(90)));
		Pose3d extenderComponentPose;
		if (DriverStation.isTeleopEnabled()) {
			extenderComponentPose = new Pose3d(0.28, 0, 0.15, new Rotation3d(0, 90 ,0)); // TODO: Update when Extender is implemented to reflect true angle
		} else {
			extenderComponentPose = new Pose3d(0.28, 0, 0.15, new Rotation3d(0, 0, 0));
		}
		//hopper extender code TBD
		Logger.recordOutput("ComponentPoses/Final", new Pose3d[] {turretComponentPose, extenderComponentPose});

		// Update field view
		field.setRobotPose(robotPose);

		// Shooter sim: launch fuel only when shooter is ready and shoot command is active
		if (shooterSim != null) {
			shooterSim.update(shooter, shooter::isShootCommandActive, turret, hood, flywheel);
		}
		if (shooterSimVisualizer != null) {
			double hoodAngleRad = isHoodEnabled ? hood.getAngleRad() : HoodConstants.kMinAngleRad;
			double flywheelSurfaceMps = flywheel.getTargetVelocityRadsPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
			double ballExitVelMps = flywheelSurfaceMps * ShooterConstants.kFlywheelSurfaceDivider;
			shooterSimVisualizer.updateFuel(
					edu.wpi.first.units.Units.MetersPerSecond.of(ballExitVelMps),
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
	} // End updateSimulation
}
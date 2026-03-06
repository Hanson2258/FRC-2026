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

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.extender.*;
import frc.robot.subsystems.extender.Extender.ExtenderState;
import frc.robot.subsystems.agitator.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.transfer.*;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.hang.*;


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
	private boolean isVisionEnabled = false;
	private boolean isIntakeEnabled = true;
	private boolean isExtenderEnabled = true;
	private boolean isAgitatorEnabled = true;
	private boolean isTransferEnabled = true;
	private boolean isTurretEnabled = true;
	private boolean isHoodEnabled = false;
	private boolean isFlywheelEnabled = true;
	private boolean isHangEnabled = true;

	// Subsystems
	private final Drive drive;
	@SuppressWarnings("unused")
	private final Vision vision;
	private final Intake intake;
	private final Extender extender;
	private final Agitator agitator;
	private final Transfer transfer;
	private final Turret turret;
	private final Hood hood;
	private final Flywheel flywheel;
	private final Hang hang;

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
				extender = isExtenderEnabled ? new Extender(new ExtenderIOSparkMax())			   : new Extender(new ExtenderIO() {});
				agitator = isAgitatorEnabled ? new Agitator(new AgitatorIOSparkMax()) 			 : new Agitator(new AgitatorIO() {});
				transfer = isTransferEnabled ? new Transfer(new TransferIOSparkMax()) : new Transfer(new TransferIO() {});
				turret   = isTurretEnabled 	 ? new Turret(new TurretIOSparkMax()) 					 : new Turret(new TurretIO() {});
				hood     = isHoodEnabled  	 ? new Hood(new HoodIOSparkMax()) 							 : new Hood(new HoodIO() {});
				flywheel = isFlywheelEnabled ? new Flywheel(new FlywheelIOTalonFX()) 				 : new Flywheel(new FlywheelIO() {});
				hang 		 = isHangEnabled	   ? new Hang(new HangIOSparkMax()) 							 : new Hang(new HangIO() {});
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
				extender = new Extender(new ExtenderIOSim());
				agitator = new Agitator(new AgitatorIOSim());
				transfer = new Transfer(new TransferIOSim());
				turret = new Turret(new TurretIOSim());
				hood = new Hood(new HoodIOSim());
				flywheel = new Flywheel(new FlywheelIOSim());
				hang = new Hang(new HangIOSim());

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
				configureFuelSimRobot(() -> extender.getState() == ExtenderState.EXTENDED, shooterSim::intakeFuel);
				break;

			// Replayed Robot, disable IO implementations
			default:
				drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, (pose) -> {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

				// Subsystems
				intake = new Intake(new IntakeIO() {});
				extender = new Extender(new ExtenderIO() {});
				agitator = new Agitator(new AgitatorIO() {});
				transfer = new Transfer(new TransferIO() {});
				turret = new Turret(new TurretIO() {});
				hood = new Hood(new HoodIO() {});
				flywheel = new Flywheel(new FlywheelIO() {});
				hang = new Hang(new HangIO() {});
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
		turret.setAimAtTargetSupplier(() -> shootWhenReadyCommand.isScheduled());

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

		// Cycle extender: Extended → Partial → Retracted → Extended
		driverController.leftTrigger().onTrue(Commands.runOnce(() -> {
			switch (extender.getState()) {
				case EXTENDED -> extender.setPartialState();
				case PARTIAL -> extender.setRetractedState();
				case RETRACTED -> extender.setExtendedState();
			}
		}, extender));

		// Intake toggle: right bumper = intaking ↔ idle, left bumper = reversing ↔ idle
		driverController.leftBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleMode(), intake),
				Commands.runOnce(() -> intake.setReversingMode(), intake),
				() -> intake.getMode() == Intake.Mode.REVERSING));
		driverController.rightBumper().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> intake.setIdleMode(), intake),
				Commands.runOnce(() -> intake.setIntakingMode(), intake),
				() -> intake.getMode() == Intake.Mode.INTAKING));

    // Toggle face-target mode when Y button is pressed
    driverController.y().onTrue(Commands.runOnce(() -> {
      isFacingHub = !isFacingHub;
      if (isFacingHub) {
        // Reset PID controller when enabling face-target mode
        faceTargetController.reset(drive.getRotation().getRadians());
      }
    }, drive));

		// Enable Hang/ Retract mode, stop when released
		if (hang != null) {
			driverController.b().onTrue(Commands.runOnce(() -> hang.goToLevel1(), hang));
			driverController.b().onFalse(Commands.runOnce(() -> hang.setIdle(), hang));
			driverController.x().onTrue(Commands.runOnce(() -> hang.goToStored(), hang));
			driverController.x().onFalse(Commands.runOnce(() -> hang.setIdle(), hang));
		}

    // Shoot toggle: on = schedule ShootWhenReadyCommand, set Flywheel to Charging if IDLE; off = cancel (command end() idles Transfer and Agitator)
		driverController.a().onTrue(Commands.runOnce(() -> {
			if (shootWhenReadyCommand.isScheduled()) {
				CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
				if (flywheel != null ) flywheel.setState(FlywheelState.IDLE);
			} else {
				if (flywheel != null && flywheel.getState() == FlywheelState.IDLE) {
					flywheel.setState(FlywheelState.CHARGING);
				}
				CommandScheduler.getInstance().schedule(shootWhenReadyCommand);
			}
		}));

		// Reset Gyro / Odometry, or if Manual Override is true, Reset Gyro
		final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
        : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // Zero Gyro
		driverController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> isRobotCentric = !isRobotCentric, drive),
				Commands.runOnce(resetGyro, drive).ignoringDisable(true),
				() -> !driverManualOverride));


    // -------- Auto Pathfind to Target --------
    // Pathfind then follow path to outpost
    driverController.leftStick().whileTrue(DriveCommands.pathfindThenFollowPath(drive, "DriveToOutpost"));

		driverController.povUp().onTrue(DriveCommands.pathfindThenFollowPath(drive, "InnerLeftLadder"));
    driverController.povLeft().onTrue(DriveCommands.pathfindThenFollowPath(drive, "OuterLeftLadder"));
    driverController.povRight().onTrue(DriveCommands.pathfindThenFollowPath(drive, "OuterRightLadder"));
    driverController.povDown().onTrue(DriveCommands.pathfindThenFollowPath(drive, "InnerRightLadder"));


		// ------------------------------------------- Driver Manual Override -------------------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		driverController.back().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> driverManualOverride = false),
				Commands.runOnce(() -> driverManualOverride = true),
				() -> driverManualOverride));
  } // End configureDriverBindings

  /** Configure Operator controls. */
  private void configureOperatorBindings(boolean enableOperatorControls) {
    // Operator Controls Enabled
    if (!enableOperatorControls) {
			return;
		}

		
		// ------------------------------------------ Operator Manual Override ------------------------------------------
		// If Manual Override is false, become true. 
		// If true, reset encoder positions and then become false.
		operatorController.back().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> operatorManualOverride = false),
				Commands.runOnce(() -> operatorManualOverride = true),
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

		// Extender Manul Position Control
		final double extenderStepPosition = Units.degreesToRadians(5);
		// Raise Extender Position
		operatorController.leftTrigger().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPosition(extenderStepPosition);
				}), 
				new InstantCommand(), 
				() -> (operatorManualOverride && extender != null))
		);
		// Lower Extender Position
		operatorController.rightTrigger().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.stepPosition(-extenderStepPosition);
				}), 
				new InstantCommand(), 
				() -> (operatorManualOverride && extender != null))
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
		// Reset Extender and Turret Encoder, and Idle all subsystems.
		operatorController.start().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> {
					extender.resetEncoders();
					turret.resetMotorEncoder();
					idleAllSubsystems();
				}, turret, extender),
				new InstantCommand(),
				() -> (operatorManualOverride && turret != null)
			)
		);

		// Hang Manual Control (step target pot voltage, no position conversion)
		double hangStepVolts = 0.1;
		// Lower Hang (Retract - higher voltage)
		operatorController.x().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepVolts(hangStepVolts), hang),
				new InstantCommand(),
				() -> (operatorManualOverride && hang != null)
			)
		);
		// Raise Hang (Extend - lower voltage)
		operatorController.b().onTrue(
			new ConditionalCommand(
				Commands.runOnce(() -> hang.stepVolts(-hangStepVolts), hang),
				new InstantCommand(),
				() -> (operatorManualOverride && hang != null)
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
	/** Idle all Subsystems.  */
	public void idleAllSubsystems() {
		CommandScheduler.getInstance().cancel(shootWhenReadyCommand);
		intake.setIdleMode();
		agitator.setIdleMode();
		transfer.setIdleMode();
		flywheel.setState(FlywheelState.IDLE);
		if (hang != null) hang.setIdle();
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
	
	/** Configures the robot for fuel simulation. */
	private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
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
				() -> extender.getState() == ExtenderState.EXTENDED,
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
		Pose3d turretComponentPose = new Pose3d(-0.125, -0.17, 0.27, new Rotation3d(0, 0, turret.getRobotFramePosition().getRadians() + Math.toRadians(90)));
		Pose3d extenderComponentPose;
		if (DriverStation.isTeleopEnabled()) {
			extenderComponentPose = new Pose3d(0.28, 0, 0.15, new Rotation3d(0, extender.getPosition() ,0));
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
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()));
			shooterSimVisualizer.update3dPose(
					edu.wpi.first.units.Units.Radians.of(turret.getRobotFramePosition().getRadians()),
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

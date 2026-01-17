// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // Swerve Drive constants
	private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts top speed possible at 12 volts
	private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second, max angular velocity
	private final double MAX_CONTROL_SPEED = 1.6; // Max speed the driver can go in x or y in m/s
	private final double TURBO_MULTIPLE = 2; // Technically a divider for how slow it is pre-turbo since it is limited to the max control speed

  // Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(0).withRotationalDeadband(0) // don't apply deadband here, it ends up being jerky apply it in the request supplier
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	// Controllers
	private final CommandXboxController driverController = new CommandXboxController(0);
  private static final double XBOX_DEADBAND = 0.09;

  // Subsystems Intialization
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // Creates Drivetrain and configures Autobuilder settings


  // Path follower
	private final SendableChooser<Command> autoChooser;


  // Manual Override and Encoder Reset
  public static boolean manualOverride = false;
  private boolean encoderReset = false;

  // Telemetry
	private final Telemetry logger = new Telemetry(MaxSpeed);


	/**
	 * RobotContainer constructor initializes the robot.
	 */
  public RobotContainer() {
    // Register the named commands for Auto
    registerCommands();

    // Configure the trigger bindings
		configureDriveBindings(true); // False to disable driving
    configureOperatorBindings(false); // False to disable operator controls

    // Setup the auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("Drive Straight"); // Default auto program to run
		SmartDashboard.putData("Auto Mode", autoChooser);
  } // End RobotContainer


	/** 
   * Deadband function to eliminate small joystick inputs.
   * @param value to apply deadband to.
   * */
	private static double applyDeadband(double value){
		return applyDeadband(value, XBOX_DEADBAND);
	} // End applyDeadband

  /** 
   * Deadband function to eliminate small joystick inputs.
   * @param value to apply deadband to.
   * @param deadband threshold.
   * */
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
   * Prints the current pose of the robot to the console.
   */
  public void printPose(){
    Pose2d robotPose = drivetrain.getState().Pose;
    System.out.println("x " + robotPose.getX());
    System.out.println("y " + robotPose.getY());
    System.out.println("rot " + robotPose.getRotation());
  } // End printPose


  /**
   * Configure only the drive to enable or disable
   * @param enableDriving true to enable driving, false to disable
   */
  private void configureDriveBindings(boolean enableDriving){
    // Drive Enabled
    if (enableDriving){
      // Drivetrain will execute this command periodically
      drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
          drive.withVelocityX(scaleAxisWithTurbo(-driverController.getLeftY(), driverController.getRightTriggerAxis() , MAX_CONTROL_SPEED)) // Drive forward with negative Y (forward)
            .withVelocityY(scaleAxisWithTurbo(-driverController.getLeftX(), driverController.getRightTriggerAxis() , MAX_CONTROL_SPEED)) // Drive left with negative X (left)
            .withRotationalRate(scaleAxisWithTurbo(-driverController.getRightX(), driverController.getRightTriggerAxis() , MAX_ANGULAR_RATE)) // Drive counterclockwise with negative X (left)
        )
      );
      // Brake Mode - Stop robot from being moved
      //driverController.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

      // Reset the field-centric heading on Start button press
      //driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Drive Disabled
    } else {
      drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
          drive.withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
        )
      );
    }
  } // End configureDriveBindings


  /** Created only to reduce Merge Conflicts while both working on this file */
  private void configureOperatorBindings(boolean enableOperatorControls){
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
    return autoChooser.getSelected();
  } // End getAutonomousCommand

  /**
   * Register commands for use in the dashboard.
   */
	private void registerCommands() {
		// Register the commands here
  } // End registerCommands
  
} // End RobotContainer.java

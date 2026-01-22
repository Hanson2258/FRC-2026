// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO using GenericMotorController for direct voltage control.
 * This follows the Hardware Abstraction approach from MapleSim docs, calculating voltage directly
 * instead of relying on TalonFX controller's velocity control loop.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;
  
  // Store constants for access in setDriveVelocity
  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants;

  // PID controllers for velocity/position control
  private final PIDController driveController;
  private final PIDController turnController;
  
  // State tracking
  private boolean driveClosedLoopActivated = false;
  private boolean steerClosedLoopActivated = false;
  private double desiredMotorVelocityRadPerSec = 0.0;
  private double torqueFeedforwardVolts = 0.0;
  private Rotation2d desiredSteerFacing = new Rotation2d();

  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
      SwerveModuleSimulation simulation) {
        
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.moduleConstants = constants;
    this.simulation = simulation;
    
    // Use GenericMotorController for direct voltage control (Hardware Abstraction approach)
    this.driveMotor = simulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(60));
    this.steerMotor = simulation.useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(20));

    // Get regulated constants for PID gains
    var regulatedConstants = PhoenixUtil.regulateModuleConstantForSimulation(constants);
    
    // Create PID controllers using regulated gains
    this.driveController = new PIDController(
        regulatedConstants.DriveMotorGains.kP, 
        0, 
        regulatedConstants.DriveMotorGains.kD);
    
    this.turnController = new PIDController(
        regulatedConstants.SteerMotorGains.kP,
        0,
        regulatedConstants.SteerMotorGains.kD);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Register control loops to run at simulation rate (critical for accurate control)
    SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> runControlLoops());
  }
  
  /**
   * Runs control loops at simulation rate. This is called by SimulatedArena at high frequency
   * (typically 5 ticks per 20ms period = 4kHz effective rate) for accurate control.
   */
  public void runControlLoops() {
    // Run control loops if activated
    if (driveClosedLoopActivated) {
      calculateDriveControlLoops();
    } else {
      driveController.reset();
    }
    if (steerClosedLoopActivated) {
      calculateSteerControlLoops();
    } else {
      turnController.reset();
    }
  }
  
  /**
   * Calculates drive motor control using DCMotor model for accurate feedforward.
   * This matches the official MapleSim implementation for better accuracy.
   */
  private void calculateDriveControlLoops() {
    DCMotor motorModel = simulation.config.driveMotorConfigs.motor;
    
    // Calculate friction torque using motor model
    double frictionTorque = motorModel.getTorque(
        motorModel.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage))
        * Math.signum(desiredMotorVelocityRadPerSec);
    
    // Calculate velocity feedforward using motor model (more accurate than KS + KV)
    double velocityFeedforwardVolts = motorModel.getVoltage(frictionTorque, desiredMotorVelocityRadPerSec);
    double feedforwardVolts = velocityFeedforwardVolts + torqueFeedforwardVolts;
    
    // Get actual velocity from simulation (direct reading, more accurate)
    double actualVelocityRadPerSec = simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    
    // Calculate PID feedback
    // Note: actualVelocityRadPerSec is wheel velocity, desiredMotorVelocityRadPerSec is motor velocity
    // We need to convert wheel velocity to motor velocity for PID comparison
    double driveGearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
    double feedBackVolts = driveController.calculate(
        actualVelocityRadPerSec, 
        desiredMotorVelocityRadPerSec / driveGearRatio);
    
    // Combine feedforward and feedback
    double driveVoltage = feedforwardVolts + feedBackVolts;
    driveMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? driveVoltage : 0.0));
  }
  
  /**
   * Calculates steer motor control using PID.
   */
  private void calculateSteerControlLoops() {
    double steerVoltage = turnController.calculate(
        simulation.getSteerAbsoluteFacing().getRadians(), 
        desiredSteerFacing.getRadians());
    steerMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? steerVoltage : 0.0));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;

    // Read positions from simulation
    inputs.drivePositionRad = simulation.getDriveWheelFinalPosition().in(Radians);
    inputs.turnAbsolutePosition = simulation.getSteerAbsoluteFacing();
    inputs.turnPosition = simulation.getSteerAbsoluteFacing();

    // Read velocities directly from simulation (more accurate than calculating from position)
    inputs.driveVelocityRadPerSec = simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.turnVelocityRadPerSec = simulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);

    // Read applied voltage and current from simulation (most accurate - matches official implementation)
    inputs.driveAppliedVolts = simulation.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = simulation.getDriveMotorStatorCurrent().in(Amps);
    inputs.turnAppliedVolts = simulation.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = simulation.getSteerMotorStatorCurrent().in(Amps);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
        .mapToDouble(angle -> angle.in(Radians))
        .toArray();
    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    this.driveClosedLoopActivated = false;
    driveMotor.requestVoltage(Volts.of(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    this.steerClosedLoopActivated = false;
    steerMotor.requestVoltage(Volts.of(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // velocityRadPerSec is wheel velocity (rad/s), convert to motor velocity
    // This matches how the official implementation handles it
    double driveGearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
    this.desiredMotorVelocityRadPerSec = velocityRadPerSec * driveGearRatio;
    this.driveClosedLoopActivated = true;
    // Note: torqueFeedforwardVolts should be set by the calling code if needed
    // For now, we'll use 0.0 (can be enhanced later if torque feedforward is needed)
    this.torqueFeedforwardVolts = 0.0;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    this.desiredSteerFacing = rotation;
    this.steerClosedLoopActivated = true;
  }
}

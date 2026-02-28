// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at the root of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import frc.robot.simulation.FuelSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

/**
 * Simulation-only coordinator for shooter fuel: tracks stored fuel, gates intake by capacity,
 * and launches fuel onto the field via FuelSim when a timer fires (enabled only).
 */
public class ShooterSim {

  private static final int CAPACITY = 30;
  private static final double SHOOT_INTERVAL_SEC = 0.25;

  private final FuelSim fuelSim;
  private int fuelStored = 8;
  private final Timer shootTimer = new Timer();

  public ShooterSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
    shootTimer.start();
  }

  /** Current number of fuel (balls) stored in the robot. */
  public int getFuelStored() {
    return fuelStored;
  } // End getFuelStored

  /** True when the shooter can accept more fuel (for FuelSim intake gating). */
  public boolean canIntake() {
    return fuelStored < CAPACITY;
  }

  /** Called by FuelSim when a ball is intaked; increments stored count. */
  public void intakeFuel() {
    fuelStored++;
  }

  /**
   * Launch one fuel from the turret pose using current turret/hood/flywheel state.
   * Decrements fuelStored. No-op if no fuel or robot not registered with FuelSim.
   */
  public void launchFuel(Turret turret, Hood hood, Flywheel flywheel) {
    if (fuelStored == 0) return;

    fuelStored--;

    double turretYawRad = turret.getPosition().getRadians();
    double hoodAngleRad = hood.getAngleRad();
    double flywheelSurfaceMps = flywheel.getTargetVelocityRadsPerSec() * FlywheelConstants.kFlywheelRadiusMeters;
    double ballExitVelMps = flywheelSurfaceMps * ShooterConstants.kFlywheelSurfaceDivider;

    // Hood angle is from vertical; FuelSim expects elevation (0 = horizontal, 90 = up)
    double elevationRad = Math.PI / 2 - hoodAngleRad;
    fuelSim.launchFuel(
        MetersPerSecond.of(ballExitVelMps),
        Radians.of(elevationRad),
        Radians.of(turretYawRad),
        ShooterConstants.robotToTurret);
  }

  /**
   * Call from simulation periodic. Launches fuel when timer elapsed, enabled, shooter ready, and shooting active.
   */
  public void update(
      Shooter shooter,
      BooleanSupplier isShootingActive,
      Turret turret,
      Hood hood,
      Flywheel flywheel) {
    if (shootTimer.advanceIfElapsed(SHOOT_INTERVAL_SEC)
        && DriverStation.isEnabled()
        && shooter.isReadyToShoot()
        && isShootingActive.getAsBoolean()) {
      launchFuel(turret, hood, flywheel);
    }
  } // End update
}

// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at the root of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    double flywheelRadPerSec = flywheel.getTargetVelocityRadsPerSec();
    double launchSpeedMetersPerSec =
        flywheelRadPerSec * FlywheelConstants.kFlywheelRadiusMeters;
    double launchHeightMeters = ShooterConstants.robotToTurret.getZ();

    fuelSim.launchFuel(
        MetersPerSecond.of(launchSpeedMetersPerSec),
        Radians.of(hoodAngleRad),
        Radians.of(turretYawRad),
        Meters.of(launchHeightMeters));
  }

  /**
   * Call from simulation periodic. Runs shoot timer and launches fuel when elapsed (enabled only).
   */
  public void update(Turret turret, Hood hood, Flywheel flywheel) {
    if (shootTimer.advanceIfElapsed(SHOOT_INTERVAL_SEC) && DriverStation.isEnabled()) {
      launchFuel(turret, hood, flywheel);
    }
  }
}

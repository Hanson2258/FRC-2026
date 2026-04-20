package frc.robot.simulation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LocalADStarAK;

/**
 * SIM-only behavior chooser manager for full-field extra robots.
 *
 * <p>This class intentionally excludes Primary and Second-Sim because those are human-controlled robots.
 *
 * <p>Dashboard behavior strings include {@value #OPTION_CYCLE_BUMP} for the PathPlanner bump loop (alliance ↔
 * neutral legs and hub scoring when shift and fuel allow).
 */
public final class SimFullFieldExtraBehaviourSim {

	public static final String OPTION_DO_NOTHING = "Do Nothing";
	public static final String OPTION_DEFENSE_BLOCK = "Defense (Block)";
	public static final String OPTION_DEFENSE_AGGRESSIVE = "Defense (Aggressive)";
	/** Bump loop: pathfind to authored path starts, follow spline samples, hub score when shift and fuel allow. */
	public static final String OPTION_CYCLE_BUMP = "Cycle (Bump)";

	// Staggered replanning times to avoid collisions
	/** Prime replan periods per role (different primes prevent collisions). */
	private static final int[] kPrimeReplanTicks = {23, 19, 17, 13, 11};
	/** Role-specific phase offsets for the shared per-robot replan schedule. */
	private static final int[] kPrimePhaseTicks = {0, 3, 5, 7, 11};

	// Defense Block Config
	/** Proportional gain used to hold X on the trench-neutral block line. */
	private static final double kDefenseBlockPathXP = 5.0;
	/** Proportional gain used to match defender Y to the tracked robot Y. */
	private static final double kDefenseBlockPathYP = 4.2;
	/** Linear speed clamp for defense-block steering. */
	private static final double kDefenseBlockPathMaxLinearMetersPerSec = 2.5;
	/** Extra stand-off from trench neutral edge so defenders stay just outside the trench lane. */
	private static final double kDefenseBlockNeutralSideOffsetMeters = 0.6;
	/** If target moves this much, force immediate replan. */
	private static final double kDefenseBlockTargetReplanDistanceMeters = 0.25;
	/** If robot strays this far from cached path start, force immediate replan. */
	private static final double kDefenseBlockStartReplanDistanceMeters = 0.35;
	/** Waypoint tolerance to advance to next point. */
	private static final double kDefenseBlockWaypointToleranceMeters = 0.35;
	/** Path constraints used by defense block pathfinder. */
	private static final PathConstraints kDefenseBlockPathConstraints =
			new PathConstraints(5.0, 3.0, 2.5 * Math.PI, 3.0 * Math.PI);

	// Defense Aggressive Config
	/** Proportional gain used for aggressive chase/backoff X control. */
	private static final double kDefenseAggressivePathXP = 10.0;
	/** Proportional gain used for aggressive chase/backoff Y control. */
	private static final double kDefenseAggressivePathYP = 10.0;
	/** Linear speed clamp for aggressive chase/backoff steering. */
	private static final double kDefenseAggressivePathMaxLinearMetersPerSec = 3.5;
	/** Path constraints used by aggressive defense pathfinder. */
	private static final PathConstraints kDefenseAggressivePathConstraints =
			new PathConstraints(5.5, 5.0, 3.0 * Math.PI, 3.5 * Math.PI);
	/** Distance threshold to count a ram contact attempt as completed. */
	private static final double kDefenseAggressiveRamSwitchDistanceMeters = 1.0;
	/** Backoff travel distance after a ram attempt. */
	private static final double kDefenseAggressiveBackoffDistanceMeters = 2.0;
	/** Delay after ram contact before switching to backoff (1.0s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffDelayTicks = 53;
	/** If backoff cannot complete within this time, abort and charge again (5.0s @ 20ms loop). */
	private static final int kDefenseAggressiveBackoffTimeoutTicks = 250;
	/** If separation exceeds this after contact, cancel countdown and resume chase. */
	private static final double kDefenseAggressiveContactReleaseDistanceMeters = 2.2;
	/** Position tolerance to finish backoff and start next ram. */
	private static final double kDefenseAggressiveBackoffArriveMeters = 0.2;
	/** Fallback backoff heading when target and robot overlap. */
	private static final double kDefenseAggressiveFallbackBackoffXSign = -1.0;

	// Cycle (Bump): PathPlanner authored legs + navgrid to path starts
	/** AD star constraints for reaching holonomic path starts. */
	private static final PathConstraints kBumpCyclePathConstraints =
			new PathConstraints(5.0, 5.0, 2.5 * Math.PI, 3.0 * Math.PI);
	/** Translation tolerance (m) to treat holonomic path start as reached. */
	private static final double kBumpCyclePathStartArriveMeters = 0.42;
	/** Advance follow index when this close to the current sample (m). */
	private static final double kBumpCycleFollowWaypointToleranceMeters = 0.42;
	/** XY error to field velocity gain for follow and path-start approach. */
	private static final double kBumpCycleFollowLinearP = 4.5;
	/** Heading error to omega gain for follow and path-start approach. */
	private static final double kBumpCycleFollowOmegaP = 2.8;
	/** Linear speed clamp during bump follow (m/s). */
	private static final double kBumpCycleFollowMaxLinearMetersPerSec = 3.0;
	/** Omega clamp during bump follow (rad/s). */
	private static final double kBumpCycleFollowMaxOmegaRadPerSec = 2.8;
	/**
	 * Minimum sim ticks between hub launch attempts during {@link #OPTION_CYCLE_BUMP} for full-field extras only.
	 * With a 20 ms robot loop, average launch rate is about 50 divided by this value (per second).
	 * Primary and second sim use {@link frc.robot.subsystems.shooter.ShooterSim} for shoot cadence instead.
	 */
	private static final int kBumpCycleScoreLaunchIntervalTicks = 5;
	/** Max |heading error| (rad) before a launch is allowed. */
	private static final double kBumpCycleScoreFacingToleranceRad = 0.14;
	/**
	 * Sim ticks without reaching holonomic path start before re-picking pathfind leg from current field zone (2.0 s
	 * when {@link #updateExtraRobotBehaviors} runs once per 20 ms robot period).
	 */
	private static final int kBumpCyclePathfindStartTimeoutTicks = 100;

	/** Deployed PathPlanner stem for alliance-to-neutral bump leg (blue frame). */
	private static final String BUMP_PATH_ALLIANCE_TO_NEUTRAL = "Bump-AllianceToNeutral-Left";
	/** Deployed PathPlanner stem for neutral-to-alliance bump leg (blue frame). */
	private static final String BUMP_PATH_NEUTRAL_TO_ALLIANCE = "Bump-NeutralToAlliance-Right";

	/** Loaded blue-frame alliance-to-neutral path; null if load failed. */
	private static PathPlannerPath bumpA2nBlueAuthoring;
	/** Loaded blue-frame neutral-to-alliance path; null if load failed. */
	private static PathPlannerPath bumpN2aBlueAuthoring;
	/** Red alliance copy of {@link #bumpA2nBlueAuthoring}; null if load failed. */
	private static PathPlannerPath bumpA2nRedAuthoring;
	/** Red alliance copy of {@link #bumpN2aBlueAuthoring}; null if load failed. */
	private static PathPlannerPath bumpN2aRedAuthoring;
	/** True after first attempt to read bump paths from deploy. */
	private static boolean bumpPathsLoadAttempted;
	/** Non-null when bump path load threw; used for sim log only. */
	private static String bumpPathsLoadError;

	private static final int[] EXTRA_ROLES = {
			SimStartingPoseFullFieldSim.ROLE_BLUE_2,
			SimStartingPoseFullFieldSim.ROLE_BLUE_3,
			SimStartingPoseFullFieldSim.ROLE_RED_1,
			SimStartingPoseFullFieldSim.ROLE_RED_2,
			SimStartingPoseFullFieldSim.ROLE_RED_3
	};

	private static final String[] EXTRA_ROLE_NAMES = {
			"Blue-2",
			"Blue-3",
			"Red-1",
			"Red-2",
			"Red-3"
	};

	private static final String[] EXTRA_ROLE_DEFAULT_BEHAVIOR = {
			OPTION_DO_NOTHING,    // Blue-2
			OPTION_DEFENSE_BLOCK, // Blue-3
			OPTION_DEFENSE_BLOCK, // Red-1
			OPTION_DEFENSE_AGGRESSIVE, // Red-2
			OPTION_DO_NOTHING     // Red-3
	};

	private static final int[][] EXTRAS_ROLES_BY_LAYOUT = {
			EXTRA_ROLES, // Layout 0: no Second-Sim
			{
					SimStartingPoseFullFieldSim.ROLE_BLUE_3,
					SimStartingPoseFullFieldSim.ROLE_RED_1,
					SimStartingPoseFullFieldSim.ROLE_RED_2,
					SimStartingPoseFullFieldSim.ROLE_RED_3
			}, // Layout 1: Second-Sim on Blue
			{
					SimStartingPoseFullFieldSim.ROLE_BLUE_2,
					SimStartingPoseFullFieldSim.ROLE_BLUE_3,
					SimStartingPoseFullFieldSim.ROLE_RED_2,
					SimStartingPoseFullFieldSim.ROLE_RED_3
			}, // Layout 2: Second-Sim on Red
	};

	private final Map<Integer, LocalADStarAK> defensePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> defensePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> defenseLastGoalByRole = new HashMap<>();
	private final Map<Integer, LocalADStarAK> aggressivePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> aggressivePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> aggressiveLastGoalByRole = new HashMap<>();
	private final Map<Integer, Boolean> aggressiveBackingOffByRole = new HashMap<>();
	private final Map<Integer, Pose2d> aggressiveBackoffGoalByRole = new HashMap<>();
	private final Map<Integer, Integer> aggressiveBackoffStartTickByRole = new HashMap<>();
	private final Map<Integer, Integer> aggressiveRamContactTickByRole = new HashMap<>();
	private final Map<Integer, String> lastBehaviorByRole = new HashMap<>();
	private int behaviorTickCounter = 0;

	/** High-level states for one extra running {@link #OPTION_CYCLE_BUMP}. */
	private enum BumpCyclePhase {
		/** Navgrid to holonomic start of alliance-to-neutral authored path. */
		PATHFIND_TO_A2N_START,
		/** Field-centric tracking of sampled authored path points. */
		FOLLOW_A2N,
		/** Navgrid to holonomic start of neutral-to-alliance authored path. */
		PATHFIND_TO_N2A_START,
		/** Field-centric tracking of sampled authored path points. */
		FOLLOW_N2A,
		/** Turn toward hub and launch carried fuel when shift timing allows. */
		SCORE_HUB
	}

	private final Map<Integer, BumpCyclePhase> bumpCyclePhaseByRole = new HashMap<>();
	private final Map<Integer, List<Translation2d>> bumpCycleFollowPointsByRole = new HashMap<>();
	private final Map<Integer, Integer> bumpCycleFollowIndexByRole = new HashMap<>();
	private final Map<Integer, LocalADStarAK> bumpCyclePathfinderByRole = new HashMap<>();
	private final Map<Integer, PathPlannerPath> bumpCyclePathByRole = new HashMap<>();
	private final Map<Integer, Pose2d> bumpCycleLastGoalByRole = new HashMap<>();
	private final Map<Integer, Integer> bumpCycleLastLaunchTickByRole = new HashMap<>();
	/** {@link #behaviorTickCounter} when the current PATHFIND_TO_* leg began (cleared on arrive, timeout, behavior change). */
	private final Map<Integer, Integer> bumpCyclePathfindLegStartTickByRole = new HashMap<>();

	public void init() {
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			SendableChooser<String> behaviorChooser = new SendableChooser<>();
			behaviorChooser.setDefaultOption(EXTRA_ROLE_DEFAULT_BEHAVIOR[index], EXTRA_ROLE_DEFAULT_BEHAVIOR[index]);
			behaviorChooser.addOption(OPTION_DO_NOTHING, OPTION_DO_NOTHING);
			behaviorChooser.addOption(OPTION_DEFENSE_BLOCK, OPTION_DEFENSE_BLOCK);
			behaviorChooser.addOption(OPTION_DEFENSE_AGGRESSIVE, OPTION_DEFENSE_AGGRESSIVE);
			behaviorChooser.addOption(OPTION_CYCLE_BUMP, OPTION_CYCLE_BUMP);
			SmartDashboard.putData(dashboardKeyForExtraIndex(index), behaviorChooser);
		}
	} // End init

	/** Returns selected behavior for the fixed extra role. */
	public String selectedBehaviorForRole(int role) {
		int index = indexForRole(role);
		if (index < 0) {
			return OPTION_DO_NOTHING;
		}
		String ntValue = NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForExtraIndex(index))
				.getEntry("selected")
				.getString("");
		if (ntValue.isEmpty()) {
			return EXTRA_ROLE_DEFAULT_BEHAVIOR[index];
		}
		return ntValue;
	} // End selectedBehaviorForRole

	/** Resets extra behavior chooser selections when the shared sim reset button is pressed. */
	public void pollResetToDefaults(boolean simMode) {
		if (!simMode) {
			return;
		}
		if (!SmartDashboard.getBoolean("SimStartingPose/ResetToDefaults", false)) {
			return;
		}
		for (int index = 0; index < EXTRA_ROLES.length; index++) {
			forceBehaviorSelectionByIndex(index, EXTRA_ROLE_DEFAULT_BEHAVIOR[index]);
		}
	} // End pollResetToDefaults

	/** Iterates active extra robots for current layout and passes (role, robot) to consumer. */
	public void forEachActiveExtra(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			BiConsumer<Integer, SimFullFieldExtraRobot> consumer) {
		if (!extrasEnabled || extraRobotsByPool == null || consumer == null) {
			return;
		}
		int layout = layoutKey(secondSimEnabled, secondSimRedAlliance);
		int[] rolesForLayout = EXTRAS_ROLES_BY_LAYOUT[layout];
		for (int poolIdx = 0; poolIdx < rolesForLayout.length; poolIdx++) {
			if (poolIdx >= extraRobotsByPool.length) {
				break;
			}
			SimFullFieldExtraRobot extraRobot = extraRobotsByPool[poolIdx];
			if (extraRobot == null) {
				continue;
			}
			consumer.accept(rolesForLayout[poolIdx], extraRobot);
		}
	} // End forEachActiveExtra

	/**
	 * Updates extra robot behaviors.
	 *
	 * <p>Red defenders track Primary. Blue defenders track Second-Sim when it is on red, otherwise Red-2.
	 *
	 * @param fuelSim registered fuel simulation instance, or null when fuel sim is off
	 * @param fuelSimEnabled when false, {@link #OPTION_CYCLE_BUMP} scoring treats carried fuel count as zero
	 */
	public void updateExtraRobotBehaviors(
			SimFullFieldExtraRobot[] extraRobotsByPool,
			boolean extrasEnabled,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance,
			boolean teleopEnabled,
			Pose2d primaryPose,
			Pose2d secondSimPose,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		Map<Integer, SimFullFieldExtraRobot> activeExtraByRole = new HashMap<>();
		forEachActiveExtra(
				extraRobotsByPool,
				extrasEnabled,
				secondSimEnabled,
				secondSimRedAlliance,
				(role, extraRobot) -> activeExtraByRole.put(role, extraRobot));

		if (!teleopEnabled) {
			for (SimFullFieldExtraRobot extraRobot : activeExtraByRole.values()) {
				extraRobot.drive.stop();
			}
			return;
		}
		behaviorTickCounter++;

		SimFullFieldExtraRobot red2ExtraRobot = activeExtraByRole.get(SimStartingPoseFullFieldSim.ROLE_RED_2);
		Pose2d red2Pose = red2ExtraRobot != null ? red2ExtraRobot.driveSimulation.getSimulatedDriveTrainPose() : null;

		for (int role : EXTRA_ROLES) {
			SimFullFieldExtraRobot extraRobot = activeExtraByRole.get(role);
			if (extraRobot == null) {
				continue;
			}

			String selectedBehavior = selectedBehaviorForRole(role);
			Logger.recordOutput("SimFullFieldExtra/" + role + "/SelectedBehavior", selectedBehavior);
			resetRoleStateOnBehaviorChange(role, selectedBehavior);

			if (OPTION_DEFENSE_BLOCK.equals(selectedBehavior)) {
				Pose2d targetPose = defenseBlockTargetPose(role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
				if (targetPose == null) {
					extraRobot.drive.stop();
					continue;
				}
				runDefenseBlockPathfind(role, extraRobot, roleIsRedAlliance(role), targetPose);
			} else if (OPTION_DEFENSE_AGGRESSIVE.equals(selectedBehavior)) {
				Pose2d targetPose = defenseBlockTargetPose(role, primaryPose, secondSimPose, red2Pose, secondSimEnabled, secondSimRedAlliance);
				if (targetPose == null) {
					extraRobot.drive.stop();
					continue;
				}
				runDefenseAggressivePathfind(role, extraRobot, roleIsRedAlliance(role), targetPose);
			} else if (OPTION_CYCLE_BUMP.equals(selectedBehavior)) {
				runBumpCycleSim(role, extraRobot, fuelSim, fuelSimEnabled);
			} else {
				// Includes explicit Do Nothing plus any future, unimplemented options.
				extraRobot.drive.stop();
			}
		}
	} // End updateExtraRobotBehaviors

	/** 0 = no Second-Sim, 1 = Second-Sim on Blue, 2 = Second-Sim on Red. */
	private static int layoutKey(boolean secondSimEnabled, boolean secondSimRedAlliance) {
		if (!secondSimEnabled) {
			return 0;
		}
		return secondSimRedAlliance ? 2 : 1;
	} // End layoutKey

	/** Returns the tracked target pose for defense block. */
	private static Pose2d defenseBlockTargetPose(
			int role,
			Pose2d primaryPose,
			Pose2d secondSimPose,
			Pose2d red2Pose,
			boolean secondSimEnabled,
			boolean secondSimRedAlliance) {
		if (roleIsRedAlliance(role)) {
			return primaryPose;
		}
		if (secondSimEnabled && secondSimRedAlliance && secondSimPose != null) {
			return secondSimPose;
		}
		return red2Pose;
	} // End defenseBlockTargetPose

	/** Runs defense block using navgrid pathfinding with prime-tick replan staggering. */
	private void runDefenseBlockPathfind(
			int role,
			SimFullFieldExtraRobot extraRobot,
			boolean defenderIsRedAlliance,
			Pose2d trackedRobotPose) {
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		Pose2d goalPose = new Pose2d(
				trenchNeutralSideXForOpposingAlliance(defenderIsRedAlliance),
				trackedRobotPose.getY(),
				selfPose.getRotation());
		PathPlannerPath cachedPath = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kDefenseBlockPathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				defensePathfinderByRole,
				defensePathByRole,
				defenseLastGoalByRole);
		Pose2d driveTargetPose = goalPose;
		if (cachedPath != null) {
			driveTargetPose = choosePathTargetPose(cachedPath, selfPose, goalPose);
		}
		driveTowardPoseProportional(
				extraRobot,
				selfPose,
				driveTargetPose,
				kDefenseBlockPathXP,
				kDefenseBlockPathYP,
				kDefenseBlockPathMaxLinearMetersPerSec);
	} // End runDefenseBlockPathfind

	/** Runs aggressive defense by looping between ram and 3m backoff. */
	private void runDefenseAggressivePathfind(
			int role,
			SimFullFieldExtraRobot extraRobot,
			boolean defenderIsRedAlliance,
			Pose2d trackedRobotPose) {
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		boolean backingOff = aggressiveBackingOffByRole.getOrDefault(role, false);
		Pose2d goalPose;

		if (!backingOff) {
			goalPose = trackedRobotPose;
			double distanceToTarget = selfPose.getTranslation().getDistance(trackedRobotPose.getTranslation());
			int contactTick = aggressiveRamContactTickByRole.getOrDefault(role, -1);

			// Latch contact once, then run a pure timer-based delay before backoff.
			if (contactTick < 0 && distanceToTarget <= kDefenseAggressiveRamSwitchDistanceMeters) {
				contactTick = behaviorTickCounter;
				aggressiveRamContactTickByRole.put(role, contactTick);
			}

			// If target separation grows too much, cancel latch and re-acquire contact.
			if (contactTick >= 0 && distanceToTarget > kDefenseAggressiveContactReleaseDistanceMeters) {
				aggressiveRamContactTickByRole.remove(role);
				contactTick = -1;
			}

			// Countdown continues even if distance wiggles around while pushing.
			if (contactTick >= 0 && behaviorTickCounter - contactTick >= kDefenseAggressiveBackoffDelayTicks) {
				Pose2d backoffGoal = computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance);
				aggressiveBackoffGoalByRole.put(role, backoffGoal);
				aggressiveBackingOffByRole.put(role, true);
				aggressiveBackoffStartTickByRole.put(role, behaviorTickCounter);
				aggressiveRamContactTickByRole.remove(role);
				backingOff = true;
				goalPose = backoffGoal;
			}
		} else {
			goalPose = aggressiveBackoffGoalByRole.getOrDefault(
					role,
					computeAggressiveBackoffGoal(selfPose, trackedRobotPose, defenderIsRedAlliance));
			double distanceToBackoff = selfPose.getTranslation().getDistance(goalPose.getTranslation());
			int backoffStartTick = aggressiveBackoffStartTickByRole.getOrDefault(role, behaviorTickCounter);
			boolean backoffTimedOut = behaviorTickCounter - backoffStartTick >= kDefenseAggressiveBackoffTimeoutTicks;
			boolean backoffArrived = distanceToBackoff <= kDefenseAggressiveBackoffArriveMeters;
			// Arrive = finish backoff cleanly; timeout = abort stale backoff. Both resume chase.
			if (backoffArrived || backoffTimedOut) {
				aggressiveBackingOffByRole.put(role, false);
				aggressiveBackoffGoalByRole.remove(role);
				aggressiveBackoffStartTickByRole.remove(role);
				aggressiveRamContactTickByRole.remove(role);
				goalPose = trackedRobotPose;
			}
		}

		PathPlannerPath path = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kDefenseAggressivePathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				aggressivePathfinderByRole,
				aggressivePathByRole,
				aggressiveLastGoalByRole);

		Pose2d driveTargetPose = goalPose;
		if (path != null) {
			driveTargetPose = choosePathTargetPose(path, selfPose, goalPose);
		}
		driveTowardPoseProportional(
				extraRobot,
				selfPose,
				driveTargetPose,
				kDefenseAggressivePathXP,
				kDefenseAggressivePathYP,
				kDefenseAggressivePathMaxLinearMetersPerSec);
	} // End runDefenseAggressivePathfind

	/** Role-specific prime-tick schedule to spread replans across simulation ticks. */
	private boolean shouldReplanForRole(int role, int[] replanTicks, int[] phaseTicks) {
		int idx = indexForRole(role);
		if (idx < 0 || idx >= replanTicks.length || idx >= phaseTicks.length) {
			return true;
		}
		int period = replanTicks[idx];
		int phase = phaseTicks[idx] % period;
		return behaviorTickCounter % period == phase;
	} // End shouldReplanForRole

	/** Replans path when schedule/goal/start thresholds require it, then returns cached/new path. */
	private PathPlannerPath replanAndGetPath(
			int role,
			Pose2d selfPose,
			Pose2d goalPose,
			PathConstraints constraints,
			int[] replanTicks,
			int[] phaseTicks,
			Map<Integer, LocalADStarAK> pathfinderByRole,
			Map<Integer, PathPlannerPath> pathByRole,
			Map<Integer, Pose2d> lastGoalByRole) {
		LocalADStarAK pathfinder = pathfinderByRole.computeIfAbsent(role, key -> new LocalADStarAK());
		PathPlannerPath cachedPath = pathByRole.get(role);
		Pose2d lastGoal = lastGoalByRole.get(role);
		boolean scheduleReplan = shouldReplanForRole(role, replanTicks, phaseTicks);
		boolean targetMoved = lastGoal == null
				|| lastGoal.getTranslation().getDistance(goalPose.getTranslation()) > kDefenseBlockTargetReplanDistanceMeters;
		List<PathPoint> cachedPoints = cachedPath != null ? cachedPath.getAllPathPoints() : null;
		boolean robotMovedFromStart = cachedPath == null
				|| cachedPoints == null
				|| cachedPoints.isEmpty()
				|| cachedPoints.get(0).position.getDistance(selfPose.getTranslation()) > kDefenseBlockStartReplanDistanceMeters;
		if (scheduleReplan || targetMoved || robotMovedFromStart) {
			pathfinder.setStartPosition(selfPose.getTranslation());
			pathfinder.setGoalPosition(goalPose.getTranslation());
			PathPlannerPath newPath = pathfinder.getCurrentPath(
					constraints,
					new GoalEndState(0.0, Rotation2d.fromRadians(0.0)));
			if (newPath != null) {
				pathByRole.put(role, newPath);
				lastGoalByRole.put(role, goalPose);
				cachedPath = newPath;
			}
		}
		return cachedPath;
	} // End replanAndGetPath

	/**
	 * Clears defense, aggressive, and {@link #OPTION_CYCLE_BUMP} cached state for {@code role} when its selected
	 * behavior string changes.
	 */
	private void resetRoleStateOnBehaviorChange(int role, String selectedBehavior) {
		String previousBehavior = lastBehaviorByRole.get(role);
		if (selectedBehavior.equals(previousBehavior)) {
			return;
		}
		lastBehaviorByRole.put(role, selectedBehavior);
		defensePathByRole.remove(role);
		defenseLastGoalByRole.remove(role);
		aggressivePathByRole.remove(role);
		aggressiveLastGoalByRole.remove(role);
		aggressiveBackingOffByRole.remove(role);
		aggressiveBackoffGoalByRole.remove(role);
		aggressiveBackoffStartTickByRole.remove(role);
		aggressiveRamContactTickByRole.remove(role);
		bumpCyclePhaseByRole.remove(role);
		bumpCycleFollowPointsByRole.remove(role);
		bumpCycleFollowIndexByRole.remove(role);
		bumpCyclePathfinderByRole.remove(role);
		bumpCyclePathByRole.remove(role);
		bumpCycleLastGoalByRole.remove(role);
		bumpCycleLastLaunchTickByRole.remove(role);
		bumpCyclePathfindLegStartTickByRole.remove(role);
	} // End resetRoleStateOnBehaviorChange

	/** Computes a 3m backoff point away from the tracked target. */
	private static Pose2d computeAggressiveBackoffGoal(
			Pose2d selfPose,
			Pose2d trackedRobotPose,
			boolean defenderIsRedAlliance) {
		double deltaX = selfPose.getX() - trackedRobotPose.getX();
		double deltaY = selfPose.getY() - trackedRobotPose.getY();
		double distance = Math.hypot(deltaX, deltaY);
		double unitX;
		double unitY;
		if (distance > 1.0e-6) {
			unitX = deltaX / distance;
			unitY = deltaY / distance;
		} else {
			unitX = defenderIsRedAlliance ? kDefenseAggressiveFallbackBackoffXSign : -kDefenseAggressiveFallbackBackoffXSign;
			unitY = 0.0;
		}
		double goalX = selfPose.getX() + unitX * kDefenseAggressiveBackoffDistanceMeters;
		double goalY = selfPose.getY() + unitY * kDefenseAggressiveBackoffDistanceMeters;
		double clampedX = MathUtil.clamp(goalX, 0.25, FieldConstants.FIELD_LENGTH_M - 0.25);
		double clampedY = MathUtil.clamp(goalY, 0.25, FieldConstants.FIELD_WIDTH_M - 0.25);
		return new Pose2d(clampedX, clampedY, selfPose.getRotation());
	} // End computeAggressiveBackoffGoal

	/** Selects a nearby lookahead waypoint on the current path. */
	private static Pose2d choosePathTargetPose(PathPlannerPath path, Pose2d selfPose, Pose2d fallbackGoalPose) {
		List<PathPoint> points = path.getAllPathPoints();
		if (points.isEmpty()) {
			return fallbackGoalPose;
		}
		int bestIndex = 0;
		double bestDistance = Double.POSITIVE_INFINITY;
		for (int i = 0; i < points.size(); i++) {
			double distance = points.get(i).position.getDistance(selfPose.getTranslation());
			if (distance < bestDistance) {
				bestDistance = distance;
				bestIndex = i;
			}
		}
		int targetIndex = Math.min(points.size() - 1, bestIndex + 2);
		if (bestDistance < kDefenseBlockWaypointToleranceMeters) {
			targetIndex = Math.min(points.size() - 1, targetIndex + 1);
		}
		return new Pose2d(points.get(targetIndex).position, fallbackGoalPose.getRotation());
	} // End choosePathTargetPose

	/**
	 * Drives field-centrically toward a pose target using independent X/Y proportional gains and a shared linear clamp.
	 * Zero rotation output; callers that want heading control should use
	 * {@link #driveBumpCycleFieldCentric(SimFullFieldExtraRobot, Pose2d, Pose2d, double)} instead.
	 */
	private static void driveTowardPoseProportional(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double kxP,
			double kyP,
			double maxLinearMetersPerSec) {
		double xErrorMeters = targetPose.getX() - selfPose.getX();
		double yErrorMeters = targetPose.getY() - selfPose.getY();
		double vxMetersPerSec = MathUtil.clamp(xErrorMeters * kxP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(yErrorMeters * kyP, -maxLinearMetersPerSec, maxLinearMetersPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, 0.0);
	} // End driveTowardPoseProportional

	/** Returns the neutral-zone-side X of the opposing alliance trench, including a small offset. */
	private static double trenchNeutralSideXForOpposingAlliance(boolean defenderIsRedAlliance) {
		double blueNeutralSideX = FieldConstants.TRENCH_BUMP_X_M
				+ (FieldConstants.TRENCH_BUMP_LENGTH_M / 2.0)
				+ kDefenseBlockNeutralSideOffsetMeters;
		if (defenderIsRedAlliance) {
			return blueNeutralSideX;
		}
		return FieldConstants.FIELD_LENGTH_M - blueNeutralSideX;
	} // End trenchNeutralSideXForOpposingAlliance

	/** True when the fixed extra role belongs to red alliance. */
	private static boolean roleIsRedAlliance(int role) {
		return role == SimStartingPoseFullFieldSim.ROLE_RED_1
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_2
				|| role == SimStartingPoseFullFieldSim.ROLE_RED_3;
	} // End roleIsRedAlliance

	/**
	 * Drops pathfind/follow state and forces {@link BumpCyclePhase#SCORE_HUB}. Caller must only invoke when alliance
	 * zone, hub shift, and carried fuel gate all passed in {@link #runBumpCycleSim}.
	 */
	private void enterBumpCycleScoreHubFromTravel(int role, SimFullFieldExtraRobot extraRobot) {
		extraRobot.drive.stop();
		bumpCyclePathfindLegStartTickByRole.remove(role);
		clearCycleNav(role);
		bumpCycleFollowPointsByRole.remove(role);
		bumpCycleFollowIndexByRole.remove(role);
		bumpCyclePathfinderByRole.remove(role);
		bumpCycleLastLaunchTickByRole.remove(role);
		bumpCyclePhaseByRole.put(role, BumpCyclePhase.SCORE_HUB);
	} // End enterBumpCycleScoreHubFromTravel

	/**
	 * Clears follow/nav state and sets pathfind phase from current X: alliance strip → {@link
	 * BumpCyclePhase#PATHFIND_TO_A2N_START}, otherwise {@link BumpCyclePhase#PATHFIND_TO_N2A_START} (same rule as cold
	 * start).
	 */
	private void bumpCycleBeginPathfindFromCurrentZone(int role, SimFullFieldExtraRobot extraRobot, boolean redAlliance) {
		bumpCyclePathfindLegStartTickByRole.remove(role);
		clearCycleNav(role);
		bumpCycleFollowPointsByRole.remove(role);
		bumpCycleFollowIndexByRole.remove(role);
		bumpCyclePathfinderByRole.remove(role);
		bumpCycleLastLaunchTickByRole.remove(role);
		Pose2d pose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		BumpCyclePhase next = AllianceUtil.isInAllianceZone(pose.getX(), redAlliance)
				? BumpCyclePhase.PATHFIND_TO_A2N_START
				: BumpCyclePhase.PATHFIND_TO_N2A_START;
		bumpCyclePhaseByRole.put(role, next);
	} // End bumpCycleBeginPathfindFromCurrentZone

	/**
	 * Leaves {@link BumpCyclePhase#SCORE_HUB} when alliance zone or hub shift gate fails; resumes pathfind from zone.
	 */
	private void exitBumpCycleScoreHubToBumpCycle(int role, SimFullFieldExtraRobot extraRobot, boolean redAlliance) {
		extraRobot.drive.stop();
		bumpCycleBeginPathfindFromCurrentZone(role, extraRobot, redAlliance);
	} // End exitBumpCycleScoreHubToBumpCycle

	/**
	 * Loads bump PathPlanner paths from deploy once; sets {@link #bumpPathsLoadError} on failure.
	 */
	private static void ensureBumpCyclePathsLoaded() {
		if (bumpPathsLoadAttempted) {
			return;
		}
		synchronized (SimFullFieldExtraBehaviourSim.class) {
			if (bumpPathsLoadAttempted) {
				return;
			}
			bumpPathsLoadAttempted = true;
			try {
				bumpA2nBlueAuthoring = PathPlannerPath.fromPathFile(BUMP_PATH_ALLIANCE_TO_NEUTRAL);
				bumpN2aBlueAuthoring = PathPlannerPath.fromPathFile(BUMP_PATH_NEUTRAL_TO_ALLIANCE);
				bumpA2nRedAuthoring = bumpA2nBlueAuthoring.flipPath();
				bumpN2aRedAuthoring = bumpN2aBlueAuthoring.flipPath();
			} catch (Exception ex) {
				bumpPathsLoadError = ex.getMessage();
				DriverStation.reportError("SimFullFieldExtra bump paths: " + ex.getMessage(), false);
			}
		}
	} // End ensureBumpCyclePathsLoaded

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached alliance-to-neutral path, or null if paths never loaded
	 */
	private static PathPlannerPath bumpA2nForAlliance(boolean redAlliance) {
		return redAlliance ? bumpA2nRedAuthoring : bumpA2nBlueAuthoring;
	} // End bumpA2nForAlliance

	/**
	 * @param redAlliance true for red alliance field frame
	 * @return cached neutral-to-alliance path, or null if paths never loaded
	 */
	private static PathPlannerPath bumpN2aForAlliance(boolean redAlliance) {
		return redAlliance ? bumpN2aRedAuthoring : bumpN2aBlueAuthoring;
	} // End bumpN2aForAlliance

	/**
	 * Runs one tick of {@link #OPTION_CYCLE_BUMP}: pathfind to path starts, follow authored samples, then hub
	 * scoring when allowed.
	 *
	 * @param role fixed extra role id
	 * @param extraRobot drive and pose for this extra
	 * @param fuelSim fuel simulation, or null when disabled
	 * @param fuelSimEnabled when false, {@link #runBumpHubScoringTick} no-ops inside
	 */
	private void runBumpCycleSim(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled) {
		ensureBumpCyclePathsLoaded();
		boolean red = roleIsRedAlliance(role);
		PathPlannerPath a2n = bumpA2nForAlliance(red);
		PathPlannerPath n2a = bumpN2aForAlliance(red);
		if (a2n == null || n2a == null) {
			if (bumpPathsLoadError != null) {
				Logger.recordOutput("SimFullFieldExtra/" + role + "/BumpCycle/LoadError", bumpPathsLoadError);
			}
			extraRobot.drive.stop();
			return;
		}
		BumpCyclePhase phase = bumpCyclePhaseByRole.get(role);
		if (phase == null) {
			Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
			boolean inOwnAllianceZone = AllianceUtil.isInAllianceZone(selfPose.getX(), red);
			// Start at neutral leg when not already home; otherwise open with alliance-to-neutral leg.
			phase = inOwnAllianceZone
					? BumpCyclePhase.PATHFIND_TO_A2N_START
					: BumpCyclePhase.PATHFIND_TO_N2A_START;
			bumpCyclePhaseByRole.put(role, phase);
		}
		Pose2d poseForScoreHubGate = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		boolean inAllianceZoneForScoreHub =
				AllianceUtil.isInAllianceZone(poseForScoreHubGate.getX(), red);
		boolean theirHubShift = HubShiftUtil.getOfficialShiftInfoForAlliance(red).active();
		int carriedFuelForScoreHubGate = 0;
		if (fuelSimEnabled && fuelSim != null && extraRobot.fuelRobotIndex >= 0) {
			carriedFuelForScoreHubGate = fuelSim.getCarriedFuelCount(extraRobot.fuelRobotIndex);
		}
		boolean hasCarriedFuelForScoreHubGate = carriedFuelForScoreHubGate > 0;
		boolean scoreHubGate = inAllianceZoneForScoreHub && theirHubShift && hasCarriedFuelForScoreHubGate;
		if (scoreHubGate && phase != BumpCyclePhase.SCORE_HUB) {
			enterBumpCycleScoreHubFromTravel(role, extraRobot);
			phase = BumpCyclePhase.SCORE_HUB;
		} else if (!scoreHubGate && phase == BumpCyclePhase.SCORE_HUB) {
			exitBumpCycleScoreHubToBumpCycle(role, extraRobot, red);
			phase = bumpCyclePhaseByRole.get(role);
		}
		Logger.recordOutput("SimFullFieldExtra/" + role + "/BumpCycle/Phase", phase.name());

		switch (phase) {
			case PATHFIND_TO_A2N_START:
				if (pathfindTowardHolonomicPathStart(role, extraRobot, a2n)) {
					bumpCyclePathfindLegStartTickByRole.remove(role);
					clearCycleNav(role);
					beginBumpFollow(role, a2n);
					bumpCyclePhaseByRole.put(role, BumpCyclePhase.FOLLOW_A2N);
				} else {
					bumpCyclePathfindLegStartTickByRole.putIfAbsent(role, behaviorTickCounter);
					int legStart = bumpCyclePathfindLegStartTickByRole.get(role);
					if (behaviorTickCounter - legStart >= kBumpCyclePathfindStartTimeoutTicks) {
						reevaluateBumpCyclePathfindPhaseFromZone(role, extraRobot, red);
					}
				}
				break;
			case FOLLOW_A2N:
				if (advanceBumpFollow(extraRobot, role)) {
					clearCycleNav(role);
					bumpCyclePathfindLegStartTickByRole.remove(role);
					bumpCyclePhaseByRole.put(role, BumpCyclePhase.PATHFIND_TO_N2A_START);
				}
				break;
			case PATHFIND_TO_N2A_START:
				if (pathfindTowardHolonomicPathStart(role, extraRobot, n2a)) {
					bumpCyclePathfindLegStartTickByRole.remove(role);
					clearCycleNav(role);
					beginBumpFollow(role, n2a);
					bumpCyclePhaseByRole.put(role, BumpCyclePhase.FOLLOW_N2A);
				} else {
					bumpCyclePathfindLegStartTickByRole.putIfAbsent(role, behaviorTickCounter);
					int legStart = bumpCyclePathfindLegStartTickByRole.get(role);
					if (behaviorTickCounter - legStart >= kBumpCyclePathfindStartTimeoutTicks) {
						reevaluateBumpCyclePathfindPhaseFromZone(role, extraRobot, red);
					}
				}
				break;
			case FOLLOW_N2A:
				if (advanceBumpFollow(extraRobot, role)) {
					if (scoreHubGate) {
						enterBumpCycleScoreHubFromTravel(role, extraRobot);
					} else {
						extraRobot.drive.stop();
						bumpCycleBeginPathfindFromCurrentZone(role, extraRobot, red);
					}
				}
				break;
			case SCORE_HUB:
				runBumpHubScoringTick(role, extraRobot, fuelSim, fuelSimEnabled, red);
				break;
			default:
				extraRobot.drive.stop();
				break;
		}
	} // End runBumpCycleSim

	/**
	 * After {@link #kBumpCyclePathfindStartTimeoutTicks} without reaching holonomic start, clears nav cache and sets
	 * pathfind phase from {@link AllianceUtil#isInAllianceZone(double, boolean)} (same rule as first entering the
	 * bump cycle).
	 */
	private void reevaluateBumpCyclePathfindPhaseFromZone(int role, SimFullFieldExtraRobot extraRobot, boolean redAlliance) {
		clearCycleNav(role);
		bumpCyclePathfindLegStartTickByRole.remove(role);
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		boolean inOwnAllianceZone = AllianceUtil.isInAllianceZone(selfPose.getX(), redAlliance);
		BumpCyclePhase next = inOwnAllianceZone ? BumpCyclePhase.PATHFIND_TO_A2N_START : BumpCyclePhase.PATHFIND_TO_N2A_START;
		bumpCyclePhaseByRole.put(role, next);
	} // End reevaluateBumpCyclePathfindPhaseFromZone

	/** Drops cached AD-star path segments for this role so the next pathfind leg replans cleanly. */
	private void clearCycleNav(int role) {
		bumpCyclePathByRole.remove(role);
		bumpCycleLastGoalByRole.remove(role);
	} // End clearCycleNav

	/**
	 * Drives toward the holonomic start pose of an authored path using the {@link #OPTION_CYCLE_BUMP} navgrid maps.
	 *
	 * @return true when the robot translation is within {@link #kBumpCyclePathStartArriveMeters} of the start
	 */
	private boolean pathfindTowardHolonomicPathStart(int role, SimFullFieldExtraRobot extraRobot, PathPlannerPath authoredPath) {
		Pose2d goalPose = authoredPath.getStartingHolonomicPose().orElse(new Pose2d());
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		PathPlannerPath navPath = replanAndGetPath(
				role,
				selfPose,
				goalPose,
				kBumpCyclePathConstraints,
				kPrimeReplanTicks,
				kPrimePhaseTicks,
				bumpCyclePathfinderByRole,
				bumpCyclePathByRole,
				bumpCycleLastGoalByRole);
		Pose2d driveTargetPose = goalPose;
		if (navPath != null) {
			driveTargetPose = choosePathTargetPose(navPath, selfPose, goalPose);
		}
		driveBumpCycleFieldCentric(extraRobot, selfPose, driveTargetPose, goalPose.getRotation().getRadians());
		return selfPose.getTranslation().getDistance(goalPose.getTranslation()) <= kBumpCyclePathStartArriveMeters;
	} // End pathfindTowardHolonomicPathStart

	/**
	 * Copies authored path samples into follow state and resets the follow index to the first point.
	 *
	 * @param role role key for per-role follow storage
	 * @param authoredPath PathPlanner path whose samples are followed in field frame
	 */
	private void beginBumpFollow(int role, PathPlannerPath authoredPath) {
		List<Translation2d> points = new ArrayList<>();
		for (PathPoint pathPoint : authoredPath.getAllPathPoints()) {
			points.add(pathPoint.position);
		}
		bumpCycleFollowPointsByRole.put(role, points);
		bumpCycleFollowIndexByRole.put(role, 0);
	} // End beginBumpFollow

	/**
	 * Steers toward the current follow sample; advances the index when inside tolerance.
	 *
	 * @return true when the last sample is reached within tolerance, or when there are no samples
	 */
	private boolean advanceBumpFollow(SimFullFieldExtraRobot extraRobot, int role) {
		List<Translation2d> points = bumpCycleFollowPointsByRole.get(role);
		if (points == null || points.isEmpty()) {
			return true;
		}
		int index = bumpCycleFollowIndexByRole.getOrDefault(role, 0);
		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		Translation2d target = points.get(Math.min(index, points.size() - 1));
		double distance = selfPose.getTranslation().getDistance(target);
		if (distance < kBumpCycleFollowWaypointToleranceMeters) {
			if (index >= points.size() - 1) {
				return true;
			}
			bumpCycleFollowIndexByRole.put(role, index + 1);
			return false;
		}
		double headingRad = Math.atan2(target.getY() - selfPose.getY(), target.getX() - selfPose.getX());
		driveBumpCycleFieldCentric(
				extraRobot,
				selfPose,
				new Pose2d(target, Rotation2d.fromRadians(headingRad)),
				headingRad);
		return false;
	} // End advanceBumpFollow

	/**
	 * Field-centric P drive toward {@code targetPose} while tracking {@code headingGoalRad}.
	 */
	private void driveBumpCycleFieldCentric(
			SimFullFieldExtraRobot extraRobot,
			Pose2d selfPose,
			Pose2d targetPose,
			double headingGoalRad) {
		double xErrorMeters = targetPose.getX() - selfPose.getX();
		double yErrorMeters = targetPose.getY() - selfPose.getY();
		double vxMetersPerSec = MathUtil.clamp(
				xErrorMeters * kBumpCycleFollowLinearP,
				-kBumpCycleFollowMaxLinearMetersPerSec,
				kBumpCycleFollowMaxLinearMetersPerSec);
		double vyMetersPerSec = MathUtil.clamp(
				yErrorMeters * kBumpCycleFollowLinearP,
				-kBumpCycleFollowMaxLinearMetersPerSec,
				kBumpCycleFollowMaxLinearMetersPerSec);
		double headingErrorRad = MathUtil.angleModulus(headingGoalRad - selfPose.getRotation().getRadians());
		double omegaRadPerSec = MathUtil.clamp(
				headingErrorRad * kBumpCycleFollowOmegaP,
				-kBumpCycleFollowMaxOmegaRadPerSec,
				kBumpCycleFollowMaxOmegaRadPerSec);
		extraRobot.drive.driveFieldCentric(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
	} // End driveBumpCycleFieldCentric

	/**
	 * Aims the whole robot like a fixed forward turret and launches one fuel when facing, timing, and fuel state allow.
	 * Hood and exit speed follow {@link ShooterCalculator#iterativeMovingShotFromFunnelClearance} (same path as
	 * {@link frc.robot.commands.ShooterCommands#setShooterTarget} with hood enabled), without {@link
	 * frc.robot.subsystems.shooter.hood.HoodConstants} clamping.
	 *
	 * @param redAlliance selects red vs blue funnel-top hub target
	 */
	private void runBumpHubScoringTick(
			int role,
			SimFullFieldExtraRobot extraRobot,
			FuelSim fuelSim,
			boolean fuelSimEnabled,
			boolean redAlliance) {
		if (!fuelSimEnabled || fuelSim == null || extraRobot.fuelRobotIndex < 0) {
			extraRobot.drive.stop();
			return;
		}
		if (fuelSim.getCarriedFuelCount(extraRobot.fuelRobotIndex) <= 0) {
			extraRobot.drive.stop();
			return;
		}
		Pose2d pose = extraRobot.drive.getPose();
		ChassisSpeeds fieldSpeeds = extraRobot.drive.getFieldRelativeChassisSpeeds();
		Translation3d target3d =
				redAlliance ? FieldConstants.RED_FUNNEL_TOP_CENTER_3D : FieldConstants.BLUE_FUNNEL_TOP_CENTER_3D;

		double phaseDelaySec = ShooterConstants.kPhaseDelaySec;
		Pose2d estimatedPose =
				new Pose2d(
						pose.getTranslation()
								.plus(
										new Translation2d(
												fieldSpeeds.vxMetersPerSecond * phaseDelaySec,
												fieldSpeeds.vyMetersPerSecond * phaseDelaySec)),
						pose.getRotation()
								.plus(
										Rotation2d.fromRadians(
												fieldSpeeds.omegaRadiansPerSecond * phaseDelaySec)));

		ShooterCalculator.ShotData shot =
				ShooterCalculator.iterativeMovingShotFromFunnelClearance(
						estimatedPose,
						fieldSpeeds,
						target3d,
						ShooterConstants.kLookaheadIterations);

		double turretYawRobotRad =
				ShooterCalculator.calculateAzimuthAngle(estimatedPose, shot.getTarget(), 0.0).in(Radians);

		Pose2d selfPose = extraRobot.driveSimulation.getSimulatedDriveTrainPose();
		double headingGoalRad = selfPose.getRotation().getRadians() + turretYawRobotRad;
		driveBumpCycleFieldCentric(
				extraRobot,
				selfPose,
				new Pose2d(selfPose.getTranslation(), Rotation2d.fromRadians(headingGoalRad)),
				headingGoalRad);

		if (Math.abs(turretYawRobotRad) > kBumpCycleScoreFacingToleranceRad) {
			return;
		}
		int lastLaunchTick = bumpCycleLastLaunchTickByRole.getOrDefault(role, -1_000_000);
		if (behaviorTickCounter - lastLaunchTick < kBumpCycleScoreLaunchIntervalTicks) {
			return;
		}

		double exitVelMps = shot.getExitVelocity().in(MetersPerSecond);
		double flywheelSurfaceSpeedMps =
				exitVelMps
						/ ShooterConstants.kFlywheelSurfaceDivider
						* ShooterConstants.kExitVelocityCompensationMultiplier;
		double ballExitVelMps =
				flywheelSurfaceSpeedMps
						* ShooterConstants.kFlywheelSurfaceDivider
						* ShooterConstants.kSimFlywheelToFuelExitVelocityEfficiency;

		double fuelSimElevationRad = Math.PI / 2.0 - shot.getHoodAngle().in(Radians);
		fuelSim.launchFuel(
				extraRobot.fuelRobotIndex,
				MetersPerSecond.of(ballExitVelMps),
				Radians.of(fuelSimElevationRad),
				Radians.of(turretYawRobotRad),
				ShooterConstants.robotToTurret);
		bumpCycleLastLaunchTickByRole.put(role, behaviorTickCounter);
	} // End runBumpHubScoringTick

	private static int indexForRole(int role) {
		for (int i = 0; i < EXTRA_ROLES.length; i++) {
			if (EXTRA_ROLES[i] == role) {
				return i;
			}
		}
		return -1;
	} // End indexForRole

	private static String dashboardKeyForExtraIndex(int index) {
		return "SimBehavior/" + EXTRA_ROLE_NAMES[index];
	} // End dashboardKeyForExtraIndex

	private static void forceBehaviorSelectionByIndex(int index, String behavior) {
		NetworkTableInstance.getDefault()
				.getTable("SmartDashboard")
				.getSubTable(dashboardKeyForExtraIndex(index))
				.getEntry("selected")
				.setString(behavior);
	} // End forceBehaviorSelectionByIndex
} // End SimFullFieldExtraBehaviourSim

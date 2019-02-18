package org.ghrobotics.frc2019.auto.paths

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.*
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch

object TrajectoryFactory {

    /** Constraints **/

    private val kMaxVelocity = 12.feet.velocity
    private val kMaxAcceleration = 6.feet.acceleration

    private val kMaxCentripetalAccelerationElevatorUp = 4.feet.acceleration
    private val kMaxCentripetalAccelerationElevatorDown = 4.feet.acceleration

    val kMaxVoltage = 10.volt

    /** Adjusted Poses **/

    private val cargoShipFLAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFL,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.inch, 0.inch)
    )
    private val cargoShipFRAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFR,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(9.inch, (-4).inch)
    )
    private val cargoShipS1Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS1,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(-1.inch, 0.inch)
    )
    private val cargoShipS2Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS2,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.inch, 6.inch)
    )
    private val cargoShipS3Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS3,
        transform = Constants.kForwardIntakeToCenter
    )
    private val depotAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kDepotBRCorner,
        transform = Constants.kBackwardIntakeToCenter
    )
    private val loadingStationAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kLoadingStation,
        transform = Constants.kBackwardIntakeToCenter,
        translationalOffset = Translation2d(4.inch, 0.inch)
    )
    private val rocketFAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketF,
        transform = Constants.kForwardIntakeToCenter
    )
    private val rocketNAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketN,
        transform = Constants.kForwardIntakeToCenter,
        rotationalOffset = 10.degree
    )

    /** Trajectories **/

    val cargoShipFLToLoadingStation: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val adjustedLoadingStation = Pose2d(
                loadingStationAdjusted.position.translation - Translation2d(0.inch, 3.inch),
                loadingStationAdjusted.position.rotation
            )
            val waypoints = listOf(
                cargoShipFLAdjusted,
                Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
                adjustedLoadingStation.asWaypoint()
            )
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(loadingStationAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val cargoShipS1ToDepot: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints =
                listOf(cargoShipS1Adjusted, Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(), depotAdjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(depotAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val cargoShipS1ToLoadingStation: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints =
                listOf(cargoShipS1Adjusted, Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(), loadingStationAdjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(loadingStationAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val centerStartToCargoShipFL: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(TrajectoryWaypoints.kCenterStart.asWaypoint(), cargoShipFLAdjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(cargoShipFLAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, 4.feet.acceleration, kMaxVoltage)
        }

    val depotToCargoShipS2: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints =
                listOf(depotAdjusted, Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(), cargoShipS2Adjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(cargoShipS2Adjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, 4.feet.acceleration, kMaxVoltage)
        }

    val loadingStationToCargoShipFR: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(
                loadingStationAdjusted,
                Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
                cargoShipFRAdjusted
            )
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(cargoShipFRAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val loadingStationToCargoShipS2: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints =
                listOf(loadingStationAdjusted, Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(), cargoShipS2Adjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(cargoShipS2Adjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val loadingStationToRocketF: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(
                loadingStationAdjusted,
                Pose2d(17.039.feet, 6.378.feet, 9.degree).asWaypoint(),
                rocketFAdjusted
            )
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorUp),
                VelocityLimitRadiusConstraint(rocketFAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val loadingStationToRocketN: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val newRocketNAdjusted = TrajectoryWaypoints.Waypoint(
                rocketNAdjusted.position,
                translationalOffset = Translation2d(0.inch, (-2).inch),
                rotationalOffset = 10.degree
            )
            val waypoints =
                listOf(loadingStationAdjusted, newRocketNAdjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorUp),
                VelocityLimitRadiusConstraint(rocketNAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val rocketNToDepot: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(rocketNAdjusted, depotAdjusted)
            val constraints = listOf(CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown))
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, 3.feet.acceleration, kMaxVoltage)
        }

    val rocketFToDepot: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(
                rocketFAdjusted,
                Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
                depotAdjusted
            )
            val constraints = listOf(CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown))
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val rocketNToLoadingStation: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val newLoadingStation = TrajectoryWaypoints.Waypoint(
                loadingStationAdjusted.position,
                translationalOffset = Translation2d(0.inch, -3.inch)
            )
            val waypoints = listOf(rocketNAdjusted, newLoadingStation)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorDown),
                VelocityLimitRadiusConstraint(loadingStationAdjusted.position.translation, 3.feet, 4.feet.velocity)
            )
            return generateTrajectory(true, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    val sideStartToCargoShipS1: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(TrajectoryWaypoints.kSideStart.asWaypoint(), cargoShipS1Adjusted)
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorUp),
                VelocityLimitRadiusConstraint(cargoShipS1Adjusted.position.translation, 5.feet, 4.feet.velocity),
                VelocityLimitRegionConstraint(TrajectoryWaypoints.kHabitatL1Platform, 3.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, 4.feet.acceleration, kMaxVoltage)
        }

    val sideStartToRocketN: TimedTrajectory<Pose2dWithCurvature>
        get() {
            val waypoints = listOf(
                TrajectoryWaypoints.kSideStart.asWaypoint(),
                TrajectoryWaypoints.kSideStart.transformBy(Pose2d(30.inch, 0.inch, 0.degree)).asWaypoint(),
                rocketNAdjusted
            )
            val constraints = listOf(
                CentripetalAccelerationConstraint(kMaxCentripetalAccelerationElevatorUp),
                VelocityLimitRadiusConstraint(rocketNAdjusted.position.translation, 5.feet, 4.feet.velocity),
                VelocityLimitRegionConstraint(TrajectoryWaypoints.kHabitatL1Platform, 3.feet.velocity)
            )
            return generateTrajectory(false, waypoints, constraints, kMaxVelocity, kMaxAcceleration, kMaxVoltage)
        }

    /** Generation **/

    fun generateTrajectory(
        reversed: Boolean,
        points: List<TrajectoryWaypoints.Waypoint>,
        constraints: List<TimingConstraint<Pose2dWithCurvature>>,
        maxVelocity: LinearVelocity,
        maxAcceleration: LinearAcceleration,
        maxVoltage: Volt,
        optimizeCurvature: Boolean = true
    ): TimedTrajectory<Pose2dWithCurvature> {

        val driveDynamicsConstraint = DifferentialDriveDynamicsConstraint(Constants.kDriveModel, maxVoltage)
        val allConstraints = ArrayList<TimingConstraint<Pose2dWithCurvature>>()

        allConstraints.add(driveDynamicsConstraint)
        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)

        return DefaultTrajectoryGenerator.generateTrajectory(
            points.map { it.position },
            allConstraints,
            0.inch.velocity,
            0.inch.velocity,
            maxVelocity,
            maxAcceleration,
            reversed,
            optimizeCurvature
        )
    }
}
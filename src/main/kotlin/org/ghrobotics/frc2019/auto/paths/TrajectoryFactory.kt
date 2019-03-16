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

    private val kMaxHabitatVelocity = 3.feet.velocity

    private val kFirstPathMaxAcceleration = 6.feet.acceleration

    private val kVelocityRadiusConstraintRadius = 3.feet
    private val kVelocityRadiusConstraintVelocity = 3.feet.velocity

    private val kMaxCentripetalAccelerationElevatorUp = 6.feet.acceleration
    private val kMaxCentripetalAccelerationElevatorDown = 9.feet.acceleration

    private val kMaxVoltage = 10.volt

    /** Adjusted Poses **/

    private val cargoShipFLAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFL,
        transform = Constants.kForwardIntakeToCenter
    )
    private val cargoShipFRAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFR,
        transform = Constants.kForwardIntakeToCenter
    )
    private val cargoShipS1Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS1,
        transform = Constants.kForwardIntakeToCenter
    )
    private val cargoShipS2Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS2,
        transform = Constants.kForwardIntakeToCenter
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
        transform = Constants.kBackwardIntakeToCenter
    )
    private val rocketFAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketF,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.inch, (-3).inch)
    )
    private val rocketNAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketN,
        transform = Constants.kForwardIntakeToCenter
    )

    /** Trajectories **/

    val cargoShipFLToLoadingStation = generateTrajectory(
        true,
        listOf(
            cargoShipFLAdjusted,
            Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), 8.feet.velocity, 6.feet.acceleration, kMaxVoltage
    )

    val cargoShipS1ToDepot = generateTrajectory(
        true,
        listOf(
            cargoShipS1Adjusted,
            Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val cargoShipS1ToLoadingStation = generateTrajectory(
        true,
        listOf(
            cargoShipS1Adjusted,
            Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val centerStartToCargoShipFL = generateTrajectory(
        false,
        listOf(
            TrajectoryWaypoints.kCenterStart.asWaypoint(),
            cargoShipFLAdjusted
        ),
        getConstraints(false, cargoShipFLAdjusted), kMaxVelocity, 4.feet.acceleration, kMaxVoltage
    )

    val depotToCargoShipS2 = generateTrajectory(
        false,
        listOf(
            depotAdjusted,
            Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
            cargoShipS2Adjusted
        ),
        getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToCargoShipFR = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
            cargoShipFRAdjusted.position.transformBy(Pose2d((-30).inch, 0.inch)).asWaypoint(),
            cargoShipFRAdjusted
        ),
        getConstraints(false, cargoShipFRAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToCargoShipS2 = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(15.feet, 4.951.feet, 17.degree).asWaypoint(),
            cargoShipS2Adjusted
        ),
        getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToRocketF = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(17.039.feet, 6.378.feet, 9.degree).asWaypoint(),
            rocketFAdjusted
        ),
        getConstraints(true, rocketFAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToRocketN = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            rocketNAdjusted
        ),
        getConstraints(true, rocketNAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketNToDepot = generateTrajectory(
        true,
        listOf(
            rocketNAdjusted,
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketFToDepot = generateTrajectory(
        true,
        listOf(
            rocketFAdjusted,
            Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketFToLoadingStation = generateTrajectory(
        true,
        listOf(
            rocketFAdjusted,
            Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketNToLoadingStation = generateTrajectory(
        true,
        listOf(
            rocketNAdjusted,
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val sideStartToCargoShipS1 = generateTrajectory(
        false,
        listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
            cargoShipS1Adjusted
        ),
        getConstraints(true, cargoShipS1Adjusted), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    )

    val sideStartToRocketN = generateTrajectory(
        false,
        listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
            rocketNAdjusted
        ),
        getConstraints(true, rocketNAdjusted), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    )

    val sideStartToRocketF = generateTrajectory(
        false,
        listOf(
            Pose2d(TrajectoryWaypoints.kSideStart.translation).asWaypoint(),
            rocketFAdjusted
        ),
        getConstraints(false, rocketNAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    /** Generation **/

    private fun getConstraints(elevatorUp: Boolean, trajectoryEndpoint: Pose2d) =
        listOf(
            CentripetalAccelerationConstraint(
                if (elevatorUp)
                    kMaxCentripetalAccelerationElevatorUp
                else
                    kMaxCentripetalAccelerationElevatorDown
            ),
            VelocityLimitRadiusConstraint(
                trajectoryEndpoint.translation,
                kVelocityRadiusConstraintRadius,
                kVelocityRadiusConstraintVelocity
            ),
            VelocityLimitRegionConstraint(TrajectoryWaypoints.kHabitatL1Platform, kMaxHabitatVelocity)
        )

    private fun getConstraints(elevatorUp: Boolean, trajectoryEndpoint: TrajectoryWaypoints.Waypoint) =
        getConstraints(elevatorUp, trajectoryEndpoint.position)

    private fun generateTrajectory(
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

    fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
}
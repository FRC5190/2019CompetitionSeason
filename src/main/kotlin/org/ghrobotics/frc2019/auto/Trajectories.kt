/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt

object Trajectories {

    /************************************ CONSTRAINTS ************************************/

    private val kMaxVelocity = 10.0.feet.velocity
    private val kMaxAcceleration = 4.0.feet.acceleration
    private val kMaxCentripetalAcceleration = 5.0.feet.acceleration

    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(Constants.kDriveModel, 10.0.volt),
        VelocityLimitRegionConstraint(Field.kHabitatL1Platform, 3.feet.velocity)
    )

    /************************************ STARTING LOCATIONS ************************************/

    private val kStartX =
        Field.kHabitatL2RX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 -
            Constants.kHypotenuseDifferenceForRamp

    val kSideStart =
        Pose2d(
            kStartX,
            Field.kHabitatL2BY + Constants.kBumperThickness + Constants.kRobotWidth / 2.0
        )

    val kCenterStart = Pose2d(kStartX, 13.5.feet, 0.degree)

    /************************************ FIELD POSES ************************************/

    private val kRocketNAdjusted = Waypoint(
        trueLocation = Field.kRocketN,
        transform = Constants.kForwardIntakeToCenter
    )

    private val kRocketFAdjusted = Waypoint(
        trueLocation = Field.kRocketF,
        transform = Constants.kForwardIntakeToCenter
    )

    private val kRocketBayAdjusted = Waypoint(
        trueLocation = Field.kRocketBay,
        transform = Constants.kForwardIntakeToCenter
    )


    private val kCargoShipFLAdjusted = Waypoint(
        trueLocation = Field.kCargoShipFL,
        transform = Constants.kForwardIntakeToCenter
    )

    private val kCargoShipFRAdjusted = Waypoint(
        trueLocation = Field.kCargoShipFR,
        transform = Constants.kForwardIntakeToCenter
    )


    private val kLoadingStationAdjusted = Waypoint(
        trueLocation = Field.kLoadingStation,
        transform = Constants.kBackwardIntakeToCenter
    )

    private val kDepotBRCorner = Waypoint(
        trueLocation = Field.kDepotBRCorner,
        transform = Constants.kBackwardIntakeToCenter
    )

    /************************************ TRAJECTORIES ************************************/

    val sideStartToNearRocketHatch = waypoints(
        kSideStart.asWaypoint(),
        kSideStart.transformBy(Pose2d(45.inch, 0.inch, 0.degree)).asWaypoint(),
        kRocketNAdjusted
    ).generateTrajectory(false, false)

    val nearRocketHatchToLoadingStation = waypoints(
        kRocketNAdjusted,
        kLoadingStationAdjusted
    ).generateTrajectory(true)

    val loadingStationToNearRocketHatch = waypoints(
        kLoadingStationAdjusted,
        kRocketNAdjusted
    ).generateTrajectory(false)

    val loadingStationToFarRocketHatch = waypoints(
        kLoadingStationAdjusted,
        Pose2d(17.039.feet, 6.378.feet, 9.degree).asWaypoint(),
        kRocketFAdjusted
    ).generateTrajectory(false)

    val nearRocketHatchToCargoBall = waypoints(
        kRocketNAdjusted,
        kDepotBRCorner
    ).generateTrajectory(true)

    val cargoBallToRocketBay = waypoints(
        kDepotBRCorner,
        kRocketBayAdjusted
    ).generateTrajectory(false)

    val farRocketHatchToCargoBall = waypoints(
        kRocketFAdjusted,
        Pose2d(19.216.feet, 5.345.feet, 5.degree).asWaypoint(),
        kSideStart.asWaypoint()
    ).generateTrajectory(true)

    val centerStartToLeftForwardCargoShip = waypoints(
        kCenterStart.asWaypoint(),
        kCenterStart.transformBy(Pose2d(45.inch, 0.inch, 0.degree)).asWaypoint(),
        kCargoShipFLAdjusted
    ).generateTrajectory(false)

    val leftForwardCargoShipToLoadingStation = waypoints(
        kCargoShipFLAdjusted,
        Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
        kLoadingStationAdjusted
    ).generateTrajectory(true)

    val loadingStationToRightForwardCargoShip = waypoints(
        kLoadingStationAdjusted,
        Pose2d(10.6.feet, 6.614.feet, 69.degree).asWaypoint(),
        kCargoShipFRAdjusted
    ).generateTrajectory(false)


    /************************************ HELPER METHODS ************************************/

    private data class Waypoint(
        val trueLocation: Pose2d,
        val transform: Pose2d = Pose2d(),
        val translationalOffset: Translation2d = Translation2d(),
        val rotationalOffset: Rotation2d = 0.radian
    ) {

        private val trueAndTransform = trueLocation + transform

        val position = Pose2d(
            trueAndTransform.translation + translationalOffset,
            trueAndTransform.rotation + rotationalOffset
        )
    }

    private fun Pose2d.asWaypoint() = Waypoint(this)

    private fun waypoints(vararg points: Pose2d) = points.toList()
    private fun waypoints(vararg points: Waypoint) = points.map { it.position }.toList()

    private fun List<Pose2d>.generateTrajectory(reversed: Boolean, optimize: Boolean = true) =
        DefaultTrajectoryGenerator.generateTrajectory(
            wayPoints = this, constraints = kConstraints,
            startVelocity = 0.0.meter.velocity, endVelocity = 0.0.meter.velocity,
            maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = reversed,
            optimizeSplines = optimize
        )
}

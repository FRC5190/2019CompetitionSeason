/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter

object Trajectories {

    /************************************ CONSTRAINTS ************************************/

    private val kMaxVelocity = 3.0.feet.velocity
    private val kMaxAcceleration = 3.0.feet.acceleration
    private val kMaxCentripetalAcceleration = 3.0.feet.acceleration

    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(Constants.kDriveModel, 10.0.volt),
        VelocityLimitRegionConstraint(Constants.kLevel1HabitatPlatform, 3.feet.velocity)
    )

    /************************************ STARTING LOCATIONS ************************************/

    private val kStartX =
        Constants.kLevel2HabitatRightX + Constants.kBumperThickness + Constants.kRobotLength / 2.0 -
            Constants.kHypotenuseDifferenceForRamp

    val kSideStart =
        Pose2d(
            kStartX,
            Constants.kLevel2HabitatBottomY + Constants.kBumperThickness + Constants.kRobotWidth / 2.0
        )

    val kCenterStart = Pose2d(kStartX, 13.5.feet, 0.degree)

    /************************************ FIELD POSES ************************************/

    private val kNearRocketHatch = Pose2d(19.feet - 1.254.feet, 1.6.feet, (-30).degree)
    private val kFarRocketHatch = Pose2d(19.feet + 1.254.feet, 1.6.feet, (-150).degree)
    private val kRocketBay = Pose2d(19.feet, 2.35.feet, (-90).degree)
    private val kForceToNearSideRocketBay = Pose2d(19.feet, 2.35.feet, (-100).degree)

    private val kNearRocketHatchAdjusted = kNearRocketHatch + Constants.kFrontBumperToCenter
    private val kFarRocketHatchAdjusted = kFarRocketHatch + Constants.kForwardIntakeToCenter
    private val kRocketBayAdjusted = kRocketBay + Constants.kForwardIntakeToCenter
    private val kForceToNearSideRocketBayAdjusted = kForceToNearSideRocketBay + Constants.kForwardIntakeToCenter

    private val kLeftFrontCargoShip = Pose2d(18.5.feet, 13.5.feet + 0.95.feet, 0.degree)
    private val kRightFrontCargoShip = Pose2d(18.5.feet, 13.5.feet - 0.95.feet, 0.degree)

    private val kLeftFrontCargoShipAdjusted = kLeftFrontCargoShip + Constants.kForwardIntakeToCenter
    private val kRightFrontCargoShipAdjusted = kRightFrontCargoShip + Constants.kForwardIntakeToCenter

    private val kLoadingStation = Pose2d(0.0.feet, 2.2.feet, 0.degree)
    private val kDepotCargo = Pose2d(3.223.feet, 6.533.feet, (-25).degree)

    private val kLoadingStationAdjusted = kLoadingStation + Constants.kBackwardIntakeToCenter
    private val kDepotCargoAdjusted = kDepotCargo + Constants.kBackwardIntakeToCenter

    /************************************ TRAJECTORIES ************************************/

    val sideStartToNearRocketHatch = waypoints(
        kSideStart,
        Pose2d(12.0.feet, 7.496.feet, (-55).degree),
        kNearRocketHatchAdjusted
    ).generateTrajectory(false)

    val nearRocketHatchToLoadingStation = waypoints(
        kNearRocketHatchAdjusted,
        kLoadingStationAdjusted
    ).generateTrajectory(true)

    val loadingStationToNearRocketHatch = waypoints(
        kLoadingStationAdjusted,
        kNearRocketHatchAdjusted
    ).generateTrajectory(false)

    val loadingStationToFarRocketHatch = waypoints(
        kLoadingStationAdjusted,
        Pose2d(19.805.feet, 6.378.feet, 9.degree),
        kFarRocketHatchAdjusted
    ).generateTrajectory(false)

    val nearRocketHatchToCargoBall = waypoints(
        kNearRocketHatchAdjusted,
        kDepotCargoAdjusted
    ).generateTrajectory(true)

    val cargoBallToForcedNearSideRocketBay = waypoints(
        kDepotCargoAdjusted,
        kForceToNearSideRocketBayAdjusted
    ).generateTrajectory(false)

    val cargoBallToRocketBay = waypoints(
        kDepotCargoAdjusted,
        kRocketBayAdjusted
    ).generateTrajectory(false)

    val farRocketHatchToCargoBall = waypoints(
        kFarRocketHatchAdjusted,
        Pose2d(19.216.feet, 5.345.feet, 5.degree),
        kDepotCargoAdjusted
    ).generateTrajectory(true)

    val centerStartToLeftForwardCargoShip = waypoints(
        kCenterStart,
        kLeftFrontCargoShipAdjusted
    ).generateTrajectory(false)

    val leftForwardCargoShipToLoadingStation = waypoints(
        kLeftFrontCargoShipAdjusted,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kLoadingStationAdjusted
    ).generateTrajectory(true)

    val loadingStationToRightForwardCargoShip = waypoints(
        kLoadingStationAdjusted,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kRightFrontCargoShipAdjusted
    ).generateTrajectory(false)


    /************************************ HELPER METHODS ************************************/

    private fun waypoints(vararg points: Pose2d) = points.toList()

    private fun List<Pose2d>.generateTrajectory(reversed: Boolean, optimize: Boolean = true) =
        DefaultTrajectoryGenerator.generateTrajectory(
            wayPoints = this, constraints = kConstraints,
            startVelocity = 0.0.meter.velocity, endVelocity = 0.0.meter.velocity,
            maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = reversed,
            optimizeSplines = optimize
        )

}

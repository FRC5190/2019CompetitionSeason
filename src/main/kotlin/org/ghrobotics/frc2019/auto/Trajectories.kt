/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
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
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.math.pow

object Trajectories {

    /************************************ CONSTRAINTS ************************************/

    // DC Motor Transmission for the DriveSubsystem
    private val dcTransmission = DCMotorTransmission(
        1 / org.ghrobotics.frc2019.Constants.kVDrive,
        org.ghrobotics.frc2019.Constants.kWheelRadius.value.pow(2) * org.ghrobotics.frc2019.Constants.kRobotMass / (2.0 * org.ghrobotics.frc2019.Constants.kADrive),
        org.ghrobotics.frc2019.Constants.kStaticFrictionVoltage
    )

    // Differential Drive. We are defining this here so that this class can be accessed by tests
    // without having to initialize WPILib.
    val differentialDrive = DifferentialDrive(
        org.ghrobotics.frc2019.Constants.kRobotMass,
        org.ghrobotics.frc2019.Constants.kRobotMomentOfInertia,
        org.ghrobotics.frc2019.Constants.kRobotAngularDrag,
        org.ghrobotics.frc2019.Constants.kWheelRadius.value,
        org.ghrobotics.frc2019.Constants.kTrackWidth.value / 2.0,
        org.ghrobotics.frc2019.auto.Trajectories.dcTransmission,
        org.ghrobotics.frc2019.auto.Trajectories.dcTransmission
    )

    private val kMaxVelocity = 12.0.feet.velocity
    private val kMaxAcceleration = 10.0.feet.acceleration
    private val kMaxCentripetalAcceleration = 9.0.feet.acceleration

    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(org.ghrobotics.frc2019.auto.Trajectories.kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(org.ghrobotics.frc2019.auto.Trajectories.differentialDrive, 10.0.volt),
        VelocityLimitRegionConstraint(org.ghrobotics.frc2019.Constants.kLevel1Platform, 3.feet.velocity)
    )

    /************************************ STARTING LOCATIONS ************************************/

    private val kRobotStartX =
        org.ghrobotics.frc2019.Constants.kLevel1RightX - org.ghrobotics.frc2019.Constants.kBumperLength - org.ghrobotics.frc2019.Constants.kRobotWidth / 2.0

    val kSideStart =
        Pose2d(
            org.ghrobotics.frc2019.auto.Trajectories.kRobotStartX,
            org.ghrobotics.frc2019.Constants.kLevel2BottomY + org.ghrobotics.frc2019.Constants.kBumperLength + org.ghrobotics.frc2019.Constants.kRobotLength / 2.0 +
                org.ghrobotics.frc2019.Constants.kHypotenuseDifferenceForRamp,
            (-90).degree
        )

    val kCenterStart = Pose2d(org.ghrobotics.frc2019.auto.Trajectories.kRobotStartX, 13.5.feet, 0.degree)

    /************************************ FIELD ELEMENTS ************************************/

    private val kRocketCenterlineX = 19.feet
    private val kRocketHatchXOffset = 1.254.feet
    private val kRocketHatchY = 16.inch

    private val kRocketBayY = 2.35.feet

    private val kLoadingStation = Pose2d(0.0.feet, 2.2.feet, 0.degree) + org.ghrobotics.frc2019.Constants.kBackwardIntakeToCenter

    private val kForwardCargoShipX = 18.456.feet
    private val kCargoShipCenterlineY = 13.5.feet
    private val kForwardCargoShipYOffset = 0.952.feet

    /************************************ FIELD POSES ************************************/

    val kNearRocketHatch =
        Pose2d(
            org.ghrobotics.frc2019.auto.Trajectories.kRocketCenterlineX - org.ghrobotics.frc2019.auto.Trajectories.kRocketHatchXOffset,
            org.ghrobotics.frc2019.auto.Trajectories.kRocketHatchY, (-30).degree) + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter

    val kFarRocketHatch =
        Pose2d(
            org.ghrobotics.frc2019.auto.Trajectories.kRocketCenterlineX + org.ghrobotics.frc2019.auto.Trajectories.kRocketHatchXOffset + 4.inch,
            org.ghrobotics.frc2019.auto.Trajectories.kRocketHatchY - 15.inch,
            (-150).degree
        ) + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter

    val kRocketBay = Pose2d(
        org.ghrobotics.frc2019.auto.Trajectories.kRocketCenterlineX,
        org.ghrobotics.frc2019.auto.Trajectories.kRocketBayY, (-90).degree) + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter

    val kLeftForwardCargoShip =
        Pose2d(
            org.ghrobotics.frc2019.auto.Trajectories.kForwardCargoShipX,
            org.ghrobotics.frc2019.auto.Trajectories.kCargoShipCenterlineY + org.ghrobotics.frc2019.auto.Trajectories.kForwardCargoShipYOffset,
            0.degree
        ) + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter
    val kRightForwardCargoShip =
        Pose2d(
            org.ghrobotics.frc2019.auto.Trajectories.kForwardCargoShipX,
            org.ghrobotics.frc2019.auto.Trajectories.kCargoShipCenterlineY - org.ghrobotics.frc2019.auto.Trajectories.kForwardCargoShipYOffset,
            0.degree
        ) + org.ghrobotics.frc2019.Constants.kForwardIntakeToCenter

    val kBottomRightDepotBall = Pose2d(4.376.feet, 6.447.feet, (-30).degree) + org.ghrobotics.frc2019.Constants.kBackwardIntakeToCenter

    /************************************ TRAJECTORIES ************************************/

    val sideStartToNearRocketHatch =
        org.ghrobotics.frc2019.auto.Trajectories.waypoints(
            org.ghrobotics.frc2019.auto.Trajectories.kSideStart,
            Pose2d(7.74.feet, 4.708.feet, (-20).degree),
            org.ghrobotics.frc2019.auto.Trajectories.kNearRocketHatch
        ).generateTrajectory(false)

    val nearRocketHatchToLoadingStation = org.ghrobotics.frc2019.auto.Trajectories.waypoints(
        org.ghrobotics.frc2019.auto.Trajectories.kNearRocketHatch,
        org.ghrobotics.frc2019.auto.Trajectories.kLoadingStation
    ).generateTrajectory(true)

    val loadingStationToFarRocketHatch =
        org.ghrobotics.frc2019.auto.Trajectories.waypoints(
            org.ghrobotics.frc2019.auto.Trajectories.kLoadingStation,
            Pose2d(19.805.feet, 6.378.feet, 9.degree),
            org.ghrobotics.frc2019.auto.Trajectories.kFarRocketHatch
        ).generateTrajectory(false)

    val farRocketHatchToCargoBall1 = org.ghrobotics.frc2019.auto.Trajectories.waypoints(
        org.ghrobotics.frc2019.auto.Trajectories.kFarRocketHatch,
        Pose2d(19.216.feet, 5.345.feet, 5.degree),
        org.ghrobotics.frc2019.auto.Trajectories.kBottomRightDepotBall
    ).generateTrajectory(true)

    val centerStartToLeftForwardCargoShip = org.ghrobotics.frc2019.auto.Trajectories.waypoints(
        org.ghrobotics.frc2019.auto.Trajectories.kCenterStart,
        org.ghrobotics.frc2019.auto.Trajectories.kLeftForwardCargoShip
    ).generateTrajectory(false)

    val leftForwardCargoShipToLoadingStation = org.ghrobotics.frc2019.auto.Trajectories.waypoints(
        org.ghrobotics.frc2019.auto.Trajectories.kLeftForwardCargoShip,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        org.ghrobotics.frc2019.auto.Trajectories.kLoadingStation
    ).generateTrajectory(true)

    val loadingStationToRightForwardCargoShip = org.ghrobotics.frc2019.auto.Trajectories.waypoints(
        org.ghrobotics.frc2019.auto.Trajectories.kLoadingStation,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        org.ghrobotics.frc2019.auto.Trajectories.kRightForwardCargoShip
    ).generateTrajectory(false)


    /************************************ HELPER METHODS ************************************/

    private fun waypoints(vararg points: Pose2d) = points.toList()

    private fun List<Pose2d>.generateTrajectory(reversed: Boolean) = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = this, constraints = org.ghrobotics.frc2019.auto.Trajectories.kConstraints,
        startVelocity = 0.0.meter.velocity, endVelocity = 0.0.meter.velocity,
        maxVelocity = org.ghrobotics.frc2019.auto.Trajectories.kMaxVelocity, maxAcceleration = org.ghrobotics.frc2019.auto.Trajectories.kMaxAcceleration, reversed = reversed
    )

}

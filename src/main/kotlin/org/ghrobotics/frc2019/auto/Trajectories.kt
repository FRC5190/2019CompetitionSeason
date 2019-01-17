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
        1 / Constants.kDriveKv,
        Constants.kWheelRadius.value.pow(2) * Constants.kRobotMass / (2.0 * Constants.kDriveKa),
        Constants.kStaticFrictionVoltage
    )

    // Differential Drive. We are defining this here so that this class can be accessed by tests
    // without having to initialize WPILib.
    val differentialDrive = DifferentialDrive(
        Constants.kRobotMass,
        Constants.kRobotMomentOfInertia,
        Constants.kRobotAngularDrag,
        Constants.kWheelRadius.value,
        Constants.kTrackWidth.value / 2.0,
        dcTransmission,
        dcTransmission
    )

    private val kMaxVelocity = 12.0.feet.velocity
    private val kMaxAcceleration = 10.0.feet.acceleration
    private val kMaxCentripetalAcceleration = 9.0.feet.acceleration

    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(differentialDrive, 10.0.volt),
        VelocityLimitRegionConstraint(Constants.kLevel1Platform, 3.feet.velocity)
    )

    /************************************ STARTING LOCATIONS ************************************/

    private val kRobotStartX =
        Constants.kLevel1RightX - Constants.kBumperLength - Constants.kRobotWidth / 2.0

    val kSideStart =
        Pose2d(
            kRobotStartX,
            Constants.kLevel2BottomY + Constants.kBumperLength + Constants.kRobotLength / 2.0 +
                Constants.kHypotenuseDifferenceForRamp,
            (-90).degree
        )

    val kCenterStart = Pose2d(kRobotStartX, 13.5.feet, 0.degree)

    /************************************ FIELD ELEMENTS ************************************/

    private val kRocketCenterlineX = 19.feet
    private val kRocketHatchXOffset = 1.254.feet
    private val kRocketHatchY = 16.inch

    private val kRocketBayY = 2.35.feet

    private val kLoadingStation = Pose2d(0.0.feet, 2.2.feet, 0.degree) + Constants.kBackwardIntakeToCenter

    private val kForwardCargoShipX = 18.456.feet
    private val kCargoShipCenterlineY = 13.5.feet
    private val kForwardCargoShipYOffset = 0.952.feet

    /************************************ FIELD POSES ************************************/

    val kNearRocketHatch =
        Pose2d(
            kRocketCenterlineX - kRocketHatchXOffset,
            kRocketHatchY, (-30).degree) + Constants.kForwardIntakeToCenter

    val kFarRocketHatch =
        Pose2d(
            kRocketCenterlineX + kRocketHatchXOffset + 4.inch,
            kRocketHatchY - 15.inch,
            (-150).degree
        ) + Constants.kForwardIntakeToCenter

    val kRocketBay = Pose2d(
        kRocketCenterlineX,
        kRocketBayY, (-90).degree) + Constants.kForwardIntakeToCenter

    val kLeftForwardCargoShip =
        Pose2d(
            kForwardCargoShipX,
            kCargoShipCenterlineY + kForwardCargoShipYOffset,
            0.degree
        ) + Constants.kForwardIntakeToCenter
    val kRightForwardCargoShip =
        Pose2d(
            kForwardCargoShipX,
            kCargoShipCenterlineY - kForwardCargoShipYOffset,
            0.degree
        ) + Constants.kForwardIntakeToCenter

    val kBottomRightDepotBall = Pose2d(4.376.feet, 6.447.feet, (-30).degree) + Constants.kBackwardIntakeToCenter

    /************************************ TRAJECTORIES ************************************/

    val sideStartToNearRocketHatch =
        waypoints(
            kSideStart,
            Pose2d(7.74.feet, 4.708.feet, (-20).degree),
            kNearRocketHatch
        ).generateTrajectory(false)

    val nearRocketHatchToLoadingStation = waypoints(
        kNearRocketHatch,
        kLoadingStation
    ).generateTrajectory(true)

    val loadingStationToFarRocketHatch =
        waypoints(
            kLoadingStation,
            Pose2d(19.805.feet, 6.378.feet, 9.degree),
            kFarRocketHatch
        ).generateTrajectory(false)

    val farRocketHatchToCargoBall1 = waypoints(
        kFarRocketHatch,
        Pose2d(19.216.feet, 5.345.feet, 5.degree),
        kBottomRightDepotBall
    ).generateTrajectory(true)

    val centerStartToLeftForwardCargoShip = waypoints(
        kCenterStart,
        kLeftForwardCargoShip
    ).generateTrajectory(false)

    val leftForwardCargoShipToLoadingStation = waypoints(
        kLeftForwardCargoShip,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kLoadingStation
    ).generateTrajectory(true)

    val loadingStationToRightForwardCargoShip = waypoints(
        kLoadingStation,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kRightForwardCargoShip
    ).generateTrajectory(false)


    /************************************ HELPER METHODS ************************************/

    private fun waypoints(vararg points: Pose2d) = points.toList()

    private fun List<Pose2d>.generateTrajectory(reversed: Boolean) = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = this, constraints = kConstraints,
        startVelocity = 0.0.meter.velocity, endVelocity = 0.0.meter.velocity,
        maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = reversed
    )

}
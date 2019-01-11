/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.auto

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.robot.Constants
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
import kotlin.math.pow

object Trajectories {

    /************************************ CONSTRAINTS ************************************/

    // DC Motor Transmission for the DriveSubsystem
    private val dcTransmission = DCMotorTransmission(
        1 / Constants.kVDrive,
        Constants.kWheelRadius.value.pow(2) * Constants.kRobotMass / (2.0 * Constants.kADrive),
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
        Constants.kLevel2RightX + Constants.kBumperLength + Constants.kRobotLength / 2.0 -
            Constants.kHypotenuseDifferenceForRamp

    val kSideStart =
        Pose2d(kRobotStartX, Constants.kLevel2BottomY + Constants.kBumperLength + Constants.kRobotWidth / 2.0, 0.degree)

    val kCenterStart = Pose2d(kRobotStartX, 13.5.feet, 0.degree)

    /************************************ FIELD ELEMENTS ************************************/

    private val kRocketCenterlineX = 19.feet
    private val kRocketHatchXOffset = 1.254.feet
    private val kRocketHatchY = 1.568.feet

    private val kRocketBayY = 2.35.feet

    private val kLoadingStation = Pose2d(1.8.feet, 2.2.feet, 0.degree)

    private val kForwardCargoShipX = 17.15.feet
    private val kCargoShipCenterlineY = 13.5.feet
    private val kForwardCargoShipYOffset = 0.952.feet

    //************************************ FIELD POSES ************************************/

    private val kNearRocketHatch =
        Pose2d(kRocketCenterlineX - kRocketHatchXOffset, kRocketHatchY, (-30).degree) + Constants.kForwardIntakeToCenter

    private val kFarRocketHatch =
        Pose2d(
            kRocketCenterlineX + kRocketHatchXOffset,
            kRocketHatchY,
            (-150).degree
        ) + Constants.kForwardIntakeToCenter

    private val kRocketBay = Pose2d(kRocketCenterlineX, kRocketBayY, (-90).degree) + Constants.kForwardIntakeToCenter

    private val kLeftForwardCargoShip =
        Pose2d(kForwardCargoShipX, kCargoShipCenterlineY + kForwardCargoShipYOffset, 0.degree)
    private val kRightForwardCargoShip =
        Pose2d(kForwardCargoShipX, kCargoShipCenterlineY - kForwardCargoShipYOffset, 0.degree)

    private val kBottomRightDepotBall = Pose2d(3.376.feet, 6.447.feet, (-30).degree) + Constants.kBackwardIntakeToCenter

    /************************************ TRAJECTORIES ************************************/

    // Start on the side of the platform and go to the far side of the rocket
    val sideStartToFarRocket = waypoints(kSideStart, kFarRocketHatch).generateTrajectory(false)

    // Back up from the far side of the rocket and go to the loading station
    val farRocketToLoadingStation = waypoints(
        kFarRocketHatch,
        Pose2d(18.552.feet, 5.601.feet, 5.degree),
        Pose2d(5.feet, 2.2.feet, 0.degree),
        kLoadingStation
    ).generateTrajectory(true)

    // Go from the loading station to the near side of the rocket
    val loadingStationToNearRocket = waypoints(kLoadingStation, kNearRocketHatch).generateTrajectory(false)

    // Back up from near hatch of the rocket to pick up the cargo ball
    val nearRocketToCargoBall1 = waypoints(kNearRocketHatch, kBottomRightDepotBall).generateTrajectory(true)

    // Drop the cargo ball in the rocket bay
    val cargoBall1ToRocketBay = waypoints(kBottomRightDepotBall, kRocketBay).generateTrajectory(false)

    // Start in the center of the platform and go the left-forward bay of the cargo ship
    val centerStartToLeftForwardCargoShip = waypoints(kCenterStart, kLeftForwardCargoShip).generateTrajectory(false)

    // Back up from the left-forward bay of the cargo ship to the loading station
    val leftForwardCargoShipToLoadingStation = waypoints(
        kLeftForwardCargoShip,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kLoadingStation
    ).generateTrajectory(true)

    // Go from the loading station to the right-forward bay of the cargo ship
    val loadingStationToRightForwardCargoShip = waypoints(
        kLoadingStation,
        Pose2d(10.6.feet, 6.614.feet, 69.degree),
        kRightForwardCargoShip
    ).generateTrajectory(false)


    /************************************ HELPER METHODS ************************************/

    private fun waypoints(vararg points: Pose2d) = points.toList()

    private fun List<Pose2d>.generateTrajectory(reversed: Boolean) = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = this, constraints = Trajectories.kConstraints,
        startVelocity = 0.0.meter.velocity, endVelocity = 0.0.meter.velocity,
        maxVelocity = Trajectories.kMaxVelocity, maxAcceleration = Trajectories.kMaxAcceleration, reversed = reversed
    )

}

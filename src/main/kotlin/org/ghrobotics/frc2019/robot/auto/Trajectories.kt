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
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.feet
import kotlin.math.pow

object Trajectories {

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

    // Constants in Feet Per Second
    private val kMaxVelocity = 10.0.feet.velocity // TODO Find Actual Value
    private val kMaxAcceleration = 8.0.feet.acceleration // TODO Find Actual Value
    private val kMaxCentripetalAcceleration = 4.5.feet.acceleration // TODO Find Actual Value


    // Constraints
    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(differentialDrive, 10.0.volt)
    )


    // Starting Locations
    private val kRobotStartX =
        Constants.kLevel2RightX + Constants.kBumperLength + Constants.kRobotLength / 2.0 -
            Constants.kHypotenuseDifferenceForRamp

    val kSideStart =
        Pose2d(kRobotStartX, Constants.kLevel2BottomY + Constants.kBumperLength + Constants.kRobotWidth / 2.0, 0.degree)

    val kCenterStart = Pose2d(kRobotStartX, 13.5.feet, 0.degree)


    // Field Element Locations
    private val kRocketCenterlineX = 19.feet
    private val kRocketHatchXOffset = 2.359.feet

    private val kNearRocketPose = Pose2d(kRocketCenterlineX - kRocketHatchXOffset, 2.2.feet, (-30).degree)
    private val kFarRocketPose = Pose2d(kRocketCenterlineX + kRocketHatchXOffset, 2.2.feet, (-150).degree)
    private val kLoadingStationPose = Pose2d(1.8.feet, 2.2.feet, 0.degree)


    // Trajectories
    val sideStartToFarRocket = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = listOf(
            kSideStart,
            kFarRocketPose
        ),
        constraints = kConstraints, startVelocity = 0.0.feet.velocity, endVelocity = 0.0.feet.velocity,
        maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = false
    )

    val farRocketToLoadingStation = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = listOf(
            kFarRocketPose,
            Pose2d(18.552.feet, 5.601.feet, 5.degree),
            Pose2d(5.feet, 2.2.feet, 0.degree),
            kLoadingStationPose
        ),
        constraints = kConstraints, startVelocity = 0.0.feet.velocity, endVelocity = 0.0.feet.velocity,
        maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = true
    )

    val loadingStationToNearRocket = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = listOf(
            kLoadingStationPose,
            kNearRocketPose
        ),
        constraints = kConstraints, startVelocity = 0.0.feet.velocity, endVelocity = 0.0.feet.velocity,
        maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = false
    )

    val baseline = DefaultTrajectoryGenerator.generateTrajectory(
        wayPoints = listOf(
            kSideStart,
            kSideStart + Pose2d(10.feet, 0.feet)
        ),
        constraints = kConstraints, startVelocity = 0.0.feet.velocity, endVelocity = 0.0.feet.velocity,
        maxVelocity = kMaxVelocity, maxAcceleration = kMaxAcceleration, reversed = false
    )
}

/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.auto

import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.feet

object Trajectories {

    // Constants in Feet Per Second
    private val kMaxVelocity = 10.0.feet.velocity // TODO Find Actual Value
    private val kMaxAcceleration = 6.0.feet.acceleration // TODO Find Actual Value
    private val kMaxCentripetalAcceleration = 4.5.feet.acceleration // TODO Find Actual Value


    // Constraints
    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(DriveSubsystem.differentialDrive, 10.0.volt)
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
            Pose2d(11.611.feet, 9.757.feet, 0.degree),
            Pose2d(18.015.feet, 9.48.feet, (-12).degree),
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
}

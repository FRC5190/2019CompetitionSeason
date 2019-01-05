/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.auto

import org.ghrobotics.frc2019.common.Constants
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
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
    private val kMaxAcceleration = 5.0.feet.acceleration // TODO Find Actual Value
    private val kMaxCentripetalAcceleration = 4.5.feet.acceleration // TODO Find Actual Value

    // Constraints
    private val kConstraints = listOf(
        CentripetalAccelerationConstraint(kMaxCentripetalAcceleration),
        DifferentialDriveDynamicsConstraint(DriveSubsystem.differentialDrive, 10.0.volt)
    )

    private val kRobotStartX =
        Constants.kLevel2RightX + Constants.kBumperLength + Constants.kRobotLength / 2.0 -
            Constants.kHypotenuseDifferenceForRamp

    val kSideStart =
        Pose2d(
            kRobotStartX,
            Constants.kLevel2BottomY + Constants.kBumperLength + Constants.kRobotWidth / 2.0,
            0.degree
        )

    val kCenterStart =
        Pose2d(
            kRobotStartX,
            13.5.feet,
            0.degree
        )
}

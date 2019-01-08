package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.auto.Autonomous
import org.ghrobotics.frc2019.robot.auto.StartingPositions
import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.withEquals

fun baselineRoutine() = autoRoutine {
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.baseline,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
}
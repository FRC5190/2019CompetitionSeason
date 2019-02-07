package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.withEquals

fun testTrajectoriesRoutine() = autoRoutine {
    val pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)

    +DriveSubsystem.followTrajectory(Trajectories.sideStartToNearRocketHatch, pathMirrored)
    +DriveSubsystem.followTrajectory(Trajectories.nearRocketHatchToLoadingStation, pathMirrored)
    +DriveSubsystem.followTrajectory(Trajectories.loadingStationToFarRocketHatch, pathMirrored)
}
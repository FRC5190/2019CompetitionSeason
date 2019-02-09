package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.withEquals

fun testTrajectoriesRoutine() = autoRoutine {
    val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    +DriveSubsystem.followTrajectory(Trajectories.sideStartToNearRocketHatch, pathMirrored)
    +DriveSubsystem.followTrajectory(Trajectories.nearRocketHatchToLoadingStation, pathMirrored)
    +DriveSubsystem.followTrajectory(Trajectories.loadingStationToFarRocketHatch, pathMirrored)
    +DriveSubsystem.followTrajectory(Trajectories.farRocketHatchToCargoBall, pathMirrored)
}
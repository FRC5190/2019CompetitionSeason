package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.withEquals

fun highHatchesRocketRoutine() = autoRoutine {

    val pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)

    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.sideStartToNearRocketHatch,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.nearRocketHatchToLoadingStation,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.loadingStationToFarRocketHatch,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.farRocketHatchToCargoBall,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT)
    )
}
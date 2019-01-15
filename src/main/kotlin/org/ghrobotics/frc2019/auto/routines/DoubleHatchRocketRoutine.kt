package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.withEquals

fun doubleHatchRocketRoutine() = autoRoutine {
    +DriveSubsystem.followTrajectory(
        trajectory = org.ghrobotics.frc2019.auto.Trajectories.sideStartToNearRocketHatch,
        pathMirrored = org.ghrobotics.frc2019.auto.Autonomous.startingPosition.withEquals(org.ghrobotics.frc2019.auto.StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = org.ghrobotics.frc2019.auto.Trajectories.nearRocketHatchToLoadingStation,
        pathMirrored = org.ghrobotics.frc2019.auto.Autonomous.startingPosition.withEquals(org.ghrobotics.frc2019.auto.StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = org.ghrobotics.frc2019.auto.Trajectories.loadingStationToFarRocketHatch,
        pathMirrored = org.ghrobotics.frc2019.auto.Autonomous.startingPosition.withEquals(org.ghrobotics.frc2019.auto.StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = org.ghrobotics.frc2019.auto.Trajectories.farRocketHatchToCargoBall1,
        pathMirrored = org.ghrobotics.frc2019.auto.Autonomous.startingPosition.withEquals(org.ghrobotics.frc2019.auto.StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
}
package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

fun cargoShipRoutine() = autoRoutine {
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.centerStartToLeftForwardCargoShip,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.leftForwardCargoShipToLoadingStation,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.loadingStationToRightForwardCargoShip,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.rightForwardCargoShipToDepot,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
}
package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

fun forwardCargoShipRoutine() = autoRoutine {
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.centerStartToLeftForwardCargoShip,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )

//    +DriveSubsystem.followVisionAssistedTrajectory(
//        trajectory = Trajectories.centerStartToLeftForwardCargoShip,
//        mirrored = Source(false),
//        dynamicObject = VisionProcessing.currentlyTrackedObject,
//        expectedLocation = Trajectories.kLeftForwardCargoShip.translation
//    )

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
    // Go back to the loading station to get a hatch immediately after the sandstorm ends.
    // We will let the Ramsete controller correct for the error due to starting at the right crago-bay
    // instead of the left.
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.leftForwardCargoShipToLoadingStation,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )


}
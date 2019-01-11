package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.auto.Autonomous
import org.ghrobotics.frc2019.robot.auto.StartingPositions
import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
import org.ghrobotics.lib.utils.withEquals

fun doubleHatchRocketRoutine() = autoRoutine {
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.sideStartToFarRocket,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.farRocketToLoadingStation,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.loadingStationToNearRocket,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )

    /*
    +DriveSubsystem.followVisionAssistedTrajectory(
        trajectory = { Trajectories.sideStartToFarRocket },
        mirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dynamicObject = { VisionProcessing.currentlyTrackedObject },
        expectedLocation = { Trajectories.kFarRocketHatch.translation }
    )
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.farRocketToLoadingStation,
        pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dt = DriveSubsystem.kPathFollowingDt
    )
    +DriveSubsystem.followVisionAssistedTrajectory(
        trajectory = { Trajectories.loadingStationToNearRocket },
        mirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
        dynamicObject = { VisionProcessing.currentlyTrackedObject },
        expectedLocation = { Trajectories.kNearRocketHatch.translation }
    )
    */

}
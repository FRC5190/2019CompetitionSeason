package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.SuperstructureCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun doubleHatchRocketRoutine() = autoRoutine {

    // Place hatch on near side of rocket
    +parallel {
        // Drive path to near rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.sideStartToNearRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            // Wait until the path is 1 second from completion
            +DelayCommand(Trajectories.sideStartToNearRocketHatch.lastState.t - 1.second)
            // Take superstructure to place hatch
            +SuperstructureCommand.kFrontHighRocketHatch()
        }
    }


    // Pickup hatch from loading station
    +parallel {
        // Drive path to loading station
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.nearRocketHatchToLoadingStation,
            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        // Take superstructure to pickup hatch
        +SuperstructureCommand.kBackLoadingStation()
    }

    // Place hatch on far side of rocket
    +parallel {
        // Drive path to far rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.loadingStationToFarRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            // Take the superstructure to a good state to minimize the time it takes to go up later on.
            +sequential {
                +SuperstructureCommand.kFrontLoadingStation()
                +DelayCommand(100.second)
            }.withTimeout(Trajectories.loadingStationToFarRocketHatch.lastState.t - 1.second)
            // Take superstructure to full height.
            +SuperstructureCommand.kFrontHighRocketHatch()
        }
    }

    // Get ready to pickup cargo
    +parallel {
        // Drive to cargo depot.
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.farRocketHatchToCargoBall1,
            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        // Take superstructure to intaking position.
        +SuperstructureCommand.kBackLoadingStation()
    }

}
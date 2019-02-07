package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun hatchAndCargoRocketRoutine() = autoRoutine {

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Place hatch on near side of rocket
    +parallel {
        // Drive path to near rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.sideStartToNearRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            // Wait until the path is 1 second from completion
            +DelayCommand(Trajectories.sideStartToNearRocketHatch.lastState.t - 1.second)
            // Take superstructure to place hatch
            +Superstructure.kFrontHighRocketHatch.withTimeout(1.5.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)

    // Pickup hatch from loading station
    +parallel {
        // Drive path to loading station
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.nearRocketHatchToLoadingStation,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        // Take superstructure to pickup hatch
        +Superstructure.kBackLoadingStation.withTimeout(1.5.second)
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Place hatch on near rocket
    +parallel {
        // Drive path to far rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.loadingStationToNearRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            // Take the superstructure to a good state to minimize the time it takes to go up later on.
            +sequential {
                +Superstructure.kFrontLoadingStation
                +DelayCommand(100.second)
            }.withTimeout(Trajectories.loadingStationToNearRocketHatch.lastState.t - 1.second)
            // Take superstructure to full height.
            +Superstructure.kFrontMiddleRocketHatch.withTimeout(1.5.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)

    // Pickup cargo from depot
    +parallel {
        // Drive path to loading station
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.nearRocketHatchToCargoBall,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        // Take superstructure to pickup ball
        +Superstructure.kBackLoadingStation.withTimeout(1.5.second)

        +sequential {
            +ConditionCommand { ArmSubsystem.armPosition > 150.degree }
            +IntakeCargoCommand(IntakeSubsystem.Direction.HOLD)
        }
    }

    // Place ball on rocket bay
    +parallel {
        // Drive path to far rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.cargoBallToForcedNearSideRocketBay,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            // Take the superstructure to a good state to minimize the time it takes to go up later on.
            +sequential {
                +Superstructure.kFrontLoadingStation
                +DelayCommand(100.second)
            }.withTimeout(Trajectories.cargoBallToForcedNearSideRocketBay.lastState.t - 1.second)
            // Take superstructure to full height.
            +Superstructure.kFrontHighRocketCargo.withTimeout(1.5.second)
        }
    }

    +IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE)
}
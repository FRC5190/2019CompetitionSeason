package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun hatchAndCargoRocketRoutine() = autoRoutine {

    val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    +parallel {
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.sideStartToNearRocketHatch,
            pathMirrored = pathMirrored
        )
        +sequential {
            +executeFor(Trajectories.sideStartToNearRocketHatch.duration - 3.25.second, ClosedLoopArmCommand(30.degree))
            +Superstructure.kFrontHighRocketHatch.withTimeout(4.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.1.second)

    +parallel {
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.nearRocketHatchToLoadingStation,
            pathMirrored = pathMirrored
        )
        +sequential {
            +DelayCommand(0.2.second)
            +Superstructure.kBackHatchFromLoadingStation.withTimeout(4.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
    +DelayCommand(0.2.second)

    // Place hatch on near rocket
    +parallel {
        // Drive path to far rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.loadingStationToNearRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            +executeFor(
                Trajectories.loadingStationToNearRocketHatch.duration - 2.75.second,
                Superstructure.kFrontCargoFromLoadingStation
            )
            +Superstructure.kFrontMiddleRocketHatch
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.3.second)

    // Pickup cargo from depot
    +parallel {
        // Drive path to loading station
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.nearRocketHatchToCargoBall,
            pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )

        +sequential {
            +DelayCommand(0.5.second)
            // Take superstructure to pickup ball
            +Superstructure.kBackCargoIntake.withTimeout(1.5.second)

            +sequential {
                +ConditionCommand { ArmSubsystem.armPosition > 150.degree }
                +IntakeCargoCommand(IntakeSubsystem.Direction.HOLD)
            }
        }
    }

    +IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE)
}
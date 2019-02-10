package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun forwardCargoShipRoutine() = autoRoutine {

    val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    +parallel {
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.centerStartToLeftForwardCargoShip,
            pathMirrored = false,
            dt = DriveSubsystem.kPathFollowingDt
        )
        +sequential {
            +executeFor(
                Trajectories.centerStartToLeftForwardCargoShip.duration - 2.second,
                ClosedLoopArmCommand(30.degree)
            )
            +Superstructure.kFrontHatchFromLoadingStation.withTimeout(4.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.1.second)

    +parallel {
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.leftForwardCargoShipToLoadingStation,
            pathMirrored = pathMirrored
        )
        +sequential {
            +DelayCommand(0.2.second)
            +Superstructure.kBackHatchFromLoadingStation.withTimeout(4.second)
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
    +DelayCommand(0.2.second)

    +parallel {
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.loadingStationToRightForwardCargoShip,
            pathMirrored = pathMirrored
        )
        +sequential {
            +executeFor(
                Trajectories.loadingStationToRightForwardCargoShip.duration - 2.75.second,
                Superstructure.kFrontCargoFromLoadingStation
            )
            +Superstructure.kFrontHatchFromLoadingStation
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.3.second)

    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.leftForwardCargoShipToLoadingStation,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
}
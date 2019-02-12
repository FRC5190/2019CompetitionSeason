package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.Field
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second

fun forwardCargoShipRoutine() = autoRoutine {

    // Hold hatch
    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Put hatch on FL cargo ship
    +parallel {
        +DriveSubsystem.followTrajectory(Trajectories.centerStartToLeftForwardCargoShip)
        +sequential {
            +executeFor(
                Trajectories.centerStartToLeftForwardCargoShip.duration - 2.second,
                ClosedLoopArmCommand(30.degree)
            )
            +Superstructure.kFrontHatchFromLoadingStation
        }
    }

    // Release hatch
    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.1.second)

    // Go to loading station
    +parallel {
        +DriveSubsystem.followTrajectory(Trajectories.leftForwardCargoShipToLoadingStation)
        +sequential {
            +DelayCommand(0.2.second)
            +Superstructure.kBackHatchFromLoadingStation
        }
    }

    // Pickup hatch
    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Reset odometry at loading station
    +parallel {
        +DelayCommand(0.2.second)
        +ConditionalCommand(IntakeSubsystem.isHoldingHatch, InstantRunnableCommand {
            DriveSubsystem.localization.reset(Field.kLoadingStation + Constants.kBackwardIntakeToCenter)
        })
    }

    // Go to FR cargo ship
    +parallel {
        +DriveSubsystem.followTrajectory(Trajectories.loadingStationToRightForwardCargoShip)
        +sequential {
            +executeFor(
                Trajectories.loadingStationToRightForwardCargoShip.duration - 2.75.second,
                Superstructure.kFrontCargoFromLoadingStation
            )
            +Superstructure.kFrontHatchFromLoadingStation
        }
    }

    // Place hatch
    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.3.second)

    // Go back to loading station
    +DriveSubsystem.followTrajectory(
        trajectory = Trajectories.leftForwardCargoShipToLoadingStation,
        pathMirrored = false,
        dt = DriveSubsystem.kPathFollowingDt
    )
}
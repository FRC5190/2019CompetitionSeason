package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Field
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun hatchAndCargoRocketRoutine() = autoRoutine {

    val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    +parallel {
        +DriveSubsystem.followTrajectory(Trajectories.sideStartToNearRocketHatch, pathMirrored)
        +sequential {
            +executeFor(Trajectories.sideStartToNearRocketHatch.duration - 3.25.second, ClosedLoopArmCommand(30.degree))
            +Superstructure.kFrontHighRocketHatch
        }
    }

    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    +DelayCommand(0.1.second)

    +parallel {
        +DriveSubsystem.followTrajectory(Trajectories.nearRocketHatchToLoadingStation, pathMirrored)
        +sequential {
            +DelayCommand(0.2.second)
            +Superstructure.kBackHatchFromLoadingStation
        }
    }
    
    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Reset odometry at loading station
    +parallel {
        +DelayCommand(0.2.second)
        +ConditionalCommand(IntakeSubsystem.isHoldingHatch, InstantRunnableCommand {
            DriveSubsystem.localization.reset(Field.kLoadingStation + Constants.kBackwardIntakeToCenter)
        })
    }

    // Place hatch on near rocket
    +parallel {
        // Drive path to far rocket
        +DriveSubsystem.followTrajectory(Trajectories.loadingStationToNearRocketHatch, pathMirrored)
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
        +DriveSubsystem.followTrajectory(Trajectories.nearRocketHatchToCargoBall, pathMirrored)

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
}

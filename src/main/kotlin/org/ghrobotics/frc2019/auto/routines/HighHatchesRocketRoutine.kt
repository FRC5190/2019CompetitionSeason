package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

fun highHatchesRocketRoutine() = autoRoutine {

    //    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

    // Place hatch on near side of rocket
    +parallel {
        // Drive path to near rocket
        +DriveSubsystem.followTrajectory(
            trajectory = Trajectories.sideStartToNearRocketHatch,
            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
            dt = DriveSubsystem.kPathFollowingDt
        )

        +sequential {
            +executeFor(Trajectories.sideStartToNearRocketHatch.duration - 2.25.second, ClosedLoopArmCommand(30.degree))
            +Superstructure.kFrontLoadingStation.withTimeout(2.0.second)
        }
    }


//    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)

    // Pickup hatch from loading station
    +parallel {
        // Drive path to loading station
        +sequential {
            +DelayCommand(1.second)
            +DriveSubsystem.followTrajectory(
                trajectory = Trajectories.nearRocketHatchToLoadingStation,
                pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
                dt = DriveSubsystem.kPathFollowingDt
            )
        }
//        // Take superstructure to pickup hatch
        +sequential {
            +Superstructure.kBackLoadingStation.withTimeout(1.5.second)
        }
    }
//
//
////    +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
//
//    // Place hatch on far side of rocket
//    +parallel {
//        // Drive path to far rocket
//        +DriveSubsystem.followTrajectory(
//            trajectory = Trajectories.loadingStationToFarRocketHatch,
//            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
//            dt = DriveSubsystem.kPathFollowingDt
//        )
////        +sequential {
////            // Take the superstructure to a good state to minimize the time it takes to go up later on.
////            +sequential {
////                +Superstructure.kFrontLoadingStation.withTimeout(1.5.second)
////                +DelayCommand(100.second)
////            }.withTimeout(Trajectories.loadingStationToFarRocketHatch.lastState.t - 1.second)
////            // Take superstructure to full height.
////            +Superstructure.kFrontMiddleRocketHatch.withTimeout(1.5.second)
////        }
//    }
//
////    +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
//
//    // Get ready to pickup cargo
//    +parallel {
//        // Drive to cargo depot.
//        +DriveSubsystem.followTrajectory(
//            trajectory = Trajectories.farRocketHatchToCargoBall,
//            pathMirrored = Autonomous.startingPosition.withEquals(StartingPositions.LEFT),
//            dt = DriveSubsystem.kPathFollowingDt
//        )
////        // Take superstructure to intaking position.
////        +Superstructure.kBackLoadingStation.withTimeout(1.5.second)
////
////        +sequential {
////            +ConditionCommand { ArmSubsystem.armPosition > 150.degree}
////            +IntakeCargoCommand(IntakeSubsystem.Direction.HOLD)
////        }
//    }

}
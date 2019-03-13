package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class BottomRocketRoutine : AutoRoutine() {

    private val path1 = TrajectoryFactory.sideStartToRocketF
    private val path2 = TrajectoryFactory.rocketFToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToRocketN

    override val duration: Time
        get() {
            System.out.printf(
                "Path 1: %3.3f, Path 2: %3.3f, Path 3: %3.3f%n",
                path1.duration.second,
                path2.duration.second,
                path3.duration.second
            )
            return path1.duration + path2.duration + path3.duration
        }

    override val routine
        get() = sequential {
            +parallel {
                +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
                +super.followVisionAssistedTrajectory(
                    path1,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.feet, true
                )
                +sequential {
                    +DelayCommand(path1.duration - 3.second)
                    +Superstructure.kFrontHatchFromLoadingStation
                }
            }
            +parallel {
                +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
                +super.followVisionAssistedTrajectory(
                    path2,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.feet, false
                )
                +sequential {
                    +DelayCommand(path2.duration - 3.second)
                    +Superstructure.kBackHatchFromLoadingStation
                }
            }
            +parallel {
                +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
                +super.followVisionAssistedTrajectory(
                    path3,
                    Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT),
                    4.feet, false
                )
                +Superstructure.kFrontHatchFromLoadingStation
            }
            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
        }
}
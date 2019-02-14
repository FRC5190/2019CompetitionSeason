package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class HighHatchesRocketRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sideStartToRocketN
    private val path2 = TrajectoryFactory.rocketNToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToRocketF
    private val path4 = TrajectoryFactory.rocketFToDepot

    override val duration: Time
        get() = path1.duration + path2.duration + path3.duration + path4.duration

    override val routine
        get() = sequential {

            val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            +parallel {
                +DriveSubsystem.followTrajectory(path1, pathMirrored)
                +sequential {
                    +executeFor(path1.duration - 3.25.second, ClosedLoopArmCommand(30.degree))
                    +Superstructure.kFrontHighRocketHatch.withTimeout(4.second)
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.1.second)

            +parallel {
                +DriveSubsystem.followTrajectory(path2, pathMirrored)
                +sequential {
                    +DelayCommand(0.5.second)
                    +Superstructure.kBackHatchFromLoadingStation.withTimeout(4.second)
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            // Reset odometry at loading station
            +parallel {
                +DelayCommand(0.2.second)
//        +ConditionalCommand(Source(true), InstantRunnableCommand {
//            DriveSubsystem.localization.reset(Field.kLoadingStation + Constants.kBackwardIntakeToCenter)
//        })
            }

            +parallel {
                +DriveSubsystem.followTrajectory(path3, pathMirrored)
                +sequential {
                    +executeFor(
                        path3.duration - 2.75.second,
                        Superstructure.kFrontCargoFromLoadingStation
                    )
                    +Superstructure.kFrontHighRocketHatch
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.3.second)

            +parallel {
                +DriveSubsystem.followTrajectory(path4, pathMirrored)
                +sequential {
                    +DelayCommand(0.5.second)
                    +Superstructure.kFrontCargoFromLoadingStation.withTimeout(4.second)
                }
            }
        }

}
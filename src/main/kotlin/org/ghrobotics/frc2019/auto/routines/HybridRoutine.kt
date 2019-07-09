package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeCloseCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

class HybridRoutine(private val mode: Mode) : AutoRoutine() {

    enum class Mode(
        val path1: TimedTrajectory<Pose2dWithCurvature>,
        val path2: TimedTrajectory<Pose2dWithCurvature>,
        val path3: TimedTrajectory<Pose2dWithCurvature>,
        val isLeft: Boolean
    ) {
        LEFT(
            TrajectoryFactory.centerStartToCargoShipFL,
            TrajectoryFactory.cargoShipFLToLeftLoadingStation,
            TrajectoryFactory.loadingStationToRocketN.mirror(),
            true
        ),
        RIGHT(
            TrajectoryFactory.centerStartToCargoShipFR,
            TrajectoryFactory.cargoShipFRToRightLoadingStation,
            TrajectoryFactory.loadingStationToRocketN,
            false
        )
    }

    override val duration: Time
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    override val routine: FalconCommand
        get() = sequential {
            +parallel {
                +followVisionAssistedTrajectory(mode.path1, { false }, 4.feet, true)
                +sequential {
                    +DelayCommand(mode.path1.duration - 3.5.second)
                    +Superstructure.kFrontHatchFromLoadingStation.withTimeout(2.0.second)
                }
            }

            val path2 = followVisionAssistedTrajectory(mode.path2, { false }, 4.feet)

            +parallel {
                +path2
                +sequential {
                    +IntakeHatchCommand(true).withTimeout(0.5.second)
                    +IntakeCloseCommand()
                    +Superstructure.kBackHatchFromLoadingStation
                    +IntakeHatchCommand(false).withExit { path2.wrappedValue.isCompleted }
                }
            }

            +relocalize(TrajectoryWaypoints.kLoadingStation, false, Source(mode.isLeft))

            +parallel {
                // Make sure the intake is holding the hatch panel.
                +IntakeHatchCommand(false).withTimeout(0.5.second)
                // Follow the trajectory with vision correction to the near side of the rocket.
                +super.followVisionAssistedTrajectory(
                    mode.path3,
                    { false },
                    4.feet, true
                )
                // Take the superstructure to scoring height.
                +Superstructure.kFrontHatchFromLoadingStation.withTimeout(4.second)
            }

            // Part 4: Score the hatch and go to the loading station for the end of the sandstorm period.
            +parallel {
                // Score hatch.
                // Follow the trajectory to the loading station.
                +DriveSubsystem.followTrajectory(
                    TrajectoryFactory.rocketNToDepot,
                    Source(mode.isLeft)
                )
                // Take the superstructure to a position to pick up the next hatch.
                +sequential {
                    +IntakeHatchCommand(true).withTimeout(0.5.second)
                    +parallel {
                        +Superstructure.kBackCargoIntake
                        +sequential {
                            +ConditionCommand { ArmSubsystem.position.cos < 0 }
                            +IntakeCargoCommand(releasing = false)
                        }
                    }
                }
            }
        }
}
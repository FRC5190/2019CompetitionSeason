package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCloseCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class CargoShipRoutine(private val mode: CargoShipRoutine.Mode) : AutoRoutine() {

    enum class Mode(
        val path1: TimedTrajectory<Pose2dWithCurvature>,
        val path2: TimedTrajectory<Pose2dWithCurvature>,
        val path3: TimedTrajectory<Pose2dWithCurvature>
    ) {
        SIDE(
            TrajectoryFactory.sideStartToCargoShipS1,
            TrajectoryFactory.cargoShipS1ToLoadingStation,
            TrajectoryFactory.loadingStationToCargoShipS2
        ),
        FRONT(
            TrajectoryFactory.centerStartToCargoShipFL,
            TrajectoryFactory.cargoShipFLToRightLoadingStation,
            TrajectoryFactory.loadingStationToCargoShipFR
        )
    }

    private val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

    override val duration: Time
        get() = mode.path1.duration + mode.path2.duration + mode.path3.duration

    override val routine: FalconCommand
        get() = sequential {

            +parallel {
                +followVisionAssistedTrajectory(mode.path1, pathMirrored, 4.feet, true)
                +sequential {
                    +DelayCommand(mode.path1.duration - 3.5.second)
                    +Superstructure.kFrontHatchFromLoadingStation.withTimeout(2.0.second)
                }
            }

            val path2 = followVisionAssistedTrajectory(mode.path2, pathMirrored, 4.feet)

            +parallel {
                +path2
                +sequential {
                    +IntakeHatchCommand(true).withTimeout(0.5.second)
                    +IntakeCloseCommand()
                    +Superstructure.kBackHatchFromLoadingStation
                    +IntakeHatchCommand(false).withExit { path2.wrappedValue.isCompleted }
                }
            }

            +relocalize(TrajectoryWaypoints.kLoadingStation, false, pathMirrored)

            +parallel {
                +IntakeHatchCommand(false).withTimeout(0.75.second)
                +followVisionAssistedTrajectory(mode.path3, pathMirrored, 3.feet, true)
                +sequential {
                    +executeFor(2.second, Superstructure.kStowedPosition)
                    +Superstructure.kFrontHatchFromLoadingStation.withTimeout(3.second)
                }
            }

            +parallel {
                +IntakeHatchCommand(true).withTimeout(0.5.second)
                +object : FalconCommand(DriveSubsystem) {
                    override suspend fun execute() {
                        DriveSubsystem.tankDrive(-0.3, -0.3)
                    }

                    override suspend fun dispose() {
                        DriveSubsystem.zeroOutputs()
                    }
                }.withTimeout(1.second)
            }
        }

}
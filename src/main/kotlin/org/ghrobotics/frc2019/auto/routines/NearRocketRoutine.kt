package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
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

class NearRocketRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sideStartToRocketN
    private val path2 = TrajectoryFactory.rocketNToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToRocketN

    override val duration: Time
        get() = path1.duration + path2.duration + path3.duration

    override val routine
        get() = sequential {

            val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

            +parallel {
                +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
                +followVisionAssistedTrajectory(path1, pathMirrored, 5.feet)
                +sequential {
                    +DelayCommand(path1.duration - 3.5.second)
                    +Superstructure.kFrontHighRocketHatch.withTimeout(4.second)
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.1.second)
            +relocalize(TrajectoryWaypoints.kRocketN, true, pathMirrored)

            +parallel {
                +followVisionAssistedTrajectory(path2, pathMirrored, 8.feet)
                +sequential {
                    +DelayCommand(0.5.second)
                    +Superstructure.kBackHatchFromLoadingStation.withTimeout(4.second)
                }
            }

            +relocalize(TrajectoryWaypoints.kLoadingStation, false, pathMirrored)
            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            +parallel {
                +followVisionAssistedTrajectory(path3, pathMirrored, 5.feet)
                +Superstructure.kFrontHatchFromLoadingStation.withTimeout(3.second)

            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DriveSubsystem.followTrajectory(TrajectoryFactory.rocketNToLoadingStation)
        }
}

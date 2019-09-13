package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.drive.TrajectoryVisionTrackerCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map

abstract class AutoRoutine : Source<FalconCommand> {
    abstract val duration: Time
    abstract val routine: FalconCommand

    override fun invoke() = sequential {
        +InstantRunnableCommand {
            println("[AutoRoutine] Starting routine...")
            DriveSubsystem.localization.reset(Autonomous.startingPosition().pose)
        }
        +routine
    }.withExit { Robot.emergencyActive }

    protected fun executeFor(time: Time, command: FalconCommand) = sequential {
        +command
        +DelayCommand(100.second)
    }.withTimeout(time)

    protected fun followVisionAssistedTrajectory(
        originalTrajectory: TimedTrajectory<Pose2dWithCurvature>,
        pathMirrored: BooleanSource,
        radiusFromEnd: Length,
        useAbsoluteVision: Boolean = false
    ): FalconCommand = TrajectoryVisionTrackerCommand(
        pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
        radiusFromEnd,
        false
    )

    protected fun relocalize(position: Pose2d, forward: Boolean, pathMirrored: BooleanSource) = InstantRunnableCommand {
        val newPosition = Pose2d(
            pathMirrored.map(position.mirror, position)().translation,
            DriveSubsystem.localization().rotation
        ) + if (forward) Constants.kForwardIntakeToCenter else Constants.kBackwardIntakeToCenter
        DriveSubsystem.localization.reset(newPosition)
    }
}

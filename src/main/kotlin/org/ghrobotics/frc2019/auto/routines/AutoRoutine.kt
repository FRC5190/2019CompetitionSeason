package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.Source

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
}

package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second

fun autoRoutine(block: BasicCommandGroupBuilder.() -> Unit) = sequential {
    +InstantRunnableCommand {
        println("[AutoRoutine] Starting routine...")
        DriveSubsystem.localization.reset(Autonomous.startingPosition().pose)
    }
    block()
}

fun executeFor(time: Time, command: FalconCommand) = sequential {
    +command
    +DelayCommand(100.second)
}.withTimeout(time)
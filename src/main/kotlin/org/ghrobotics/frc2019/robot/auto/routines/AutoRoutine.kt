package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.auto.Autonomous
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.BasicCommandGroupBuilder
import org.ghrobotics.lib.commands.InstantRunnableCommand
import org.ghrobotics.lib.commands.sequential

fun autoRoutine(block: BasicCommandGroupBuilder.() -> Unit) = sequential {
    +InstantRunnableCommand {
        println("[AutoRoutine] Starting routine...")
        DriveSubsystem.localization.reset(Autonomous.startingPosition().pose)
    }
    block()
}
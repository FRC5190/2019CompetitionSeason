package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.PeriodicRunnableCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.second

class BaselineRoutine : AutoRoutine() {
    override val routine: FalconCommand
        get() = sequential {
            +PeriodicRunnableCommand({
                DriveSubsystem.tankDrive(0.3, 0.3)
            }, { false }).withTimeout(5.second)
        }

    override val duration = 5.second
}
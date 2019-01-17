package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.PeriodicRunnableCommand
import org.ghrobotics.lib.mathematics.units.second

fun baselineRoutine() = autoRoutine {
    // The only reason we will ever run baseline is if encoders don't work. So it makes no sense to run a trajectory.
    +PeriodicRunnableCommand({
        DriveSubsystem.tankDrive(0.3, 0.3)
    }, { false }).withTimeout(5.second)
}
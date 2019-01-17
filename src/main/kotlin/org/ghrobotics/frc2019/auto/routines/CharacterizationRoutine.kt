package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem

fun characterizationRoutine() = autoRoutine {
    +DriveSubsystem.characterizeDrive(
        Constants.kWheelRadius,
        Constants.kTrackWidth / 2.0
    )
}
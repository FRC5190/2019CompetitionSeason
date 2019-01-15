package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem

fun characterizationRoutine() = autoRoutine {
    +DriveSubsystem.characterizeDrive(
        org.ghrobotics.frc2019.Constants.kWheelRadius,
        org.ghrobotics.frc2019.Constants.kTrackWidth / 2.0
    )
}
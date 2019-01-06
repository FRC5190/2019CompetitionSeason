package org.ghrobotics.frc2019.robot.auto.routines

import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

fun characterizationRoutine() = autoRoutine {
    +DriveSubsystem.characterizeDrive(
        Constants.kWheelRadius,
        Constants.kTrackWidth / 2.0
    )
}
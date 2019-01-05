/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import org.ghrobotics.frc2019.robot.auto.AutoMode
import org.ghrobotics.frc2019.robot.auto.StartingPositions

object Network {

    val startingPositionChooser = SendableChooser<StartingPositions>()
    val autoModeChooser = SendableChooser<AutoMode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("Main Display")

    init {
        // Initialize sendable choosers
        autoModeChooser.addOption("Characterize", AutoMode.CHARACTERIZE)
        autoModeChooser.addOption("Cargo Ship", AutoMode.CARGO_SHIP)
        autoModeChooser.setDefaultOption("Rocket",  AutoMode.ROCKET)

        startingPositionChooser.setDefaultOption("Left", StartingPositions.LEFT)
        startingPositionChooser.addOption("Center", StartingPositions.CENTER)
        startingPositionChooser.addOption("Right", StartingPositions.RIGHT)

        // Put choosers on dashboard
        mainShuffleboardDisplay.add("Auto Mode", autoModeChooser)
        mainShuffleboardDisplay.add("Starting Position", startingPositionChooser)
    }
}
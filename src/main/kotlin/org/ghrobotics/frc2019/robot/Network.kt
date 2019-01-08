/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.frc2019.robot.auto.AutoMode
import org.ghrobotics.frc2019.robot.auto.StartingPositions
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.lib.utils.launchFrequency

object Network {

    val startingPositionChooser = SendableChooser<StartingPositions>()
    val autoModeChooser = SendableChooser<AutoMode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("Main Display")

    private val globalXEntry: NetworkTableEntry = mainShuffleboardDisplay.add("Robot X", 0.0).entry
    private val globalYEntry: NetworkTableEntry = mainShuffleboardDisplay.add("Robot Y", 0.0).entry
    private val globalAEntry: NetworkTableEntry = mainShuffleboardDisplay.add("Robot Angle", 0.0).entry

    private val leftPositionEntry: NetworkTableEntry = mainShuffleboardDisplay.add("Left Encoder", 0.0).entry
    private val rightPositionEntry: NetworkTableEntry = mainShuffleboardDisplay.add("Right Encoder", 0.0).entry

    init {
        // Initialize sendable choosers
        autoModeChooser.addOption("Characterize", AutoMode.CHARACTERIZE)
        autoModeChooser.addOption("Cargo Ship", AutoMode.FORWARD_CARGO_SHIP)
        autoModeChooser.addOption("Baseline", AutoMode.BASELINE)
        autoModeChooser.setDefaultOption("Rocket", AutoMode.DOUBLE_HATCH_ROCKET)

        startingPositionChooser.setDefaultOption("Left", StartingPositions.LEFT)
        startingPositionChooser.addOption("Center", StartingPositions.CENTER)
        startingPositionChooser.addOption("Right", StartingPositions.RIGHT)

        // Put choosers on dashboard
        mainShuffleboardDisplay.add("Auto Mode", autoModeChooser)
        mainShuffleboardDisplay.add("Starting Position", startingPositionChooser)

        GlobalScope.launchFrequency {
            globalXEntry.setDouble(DriveSubsystem.localization().translation.x.feet)
            globalYEntry.setDouble(DriveSubsystem.localization().translation.y.feet)
            globalAEntry.setDouble(DriveSubsystem.localization().rotation.degree)

            leftPositionEntry.setDouble(DriveSubsystem.leftMotor.getSelectedSensorPosition(0).toDouble())
            rightPositionEntry.setDouble(DriveSubsystem.rightMotor.getSelectedSensorPosition(0).toDouble())

            Shuffleboard.update()
        }
    }
}
/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.ghrobotics.frc2019.auto.AutoMode
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.robot.vision.TargetTracker
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

object Network {

    val startingPositionChooser = enumSendableChooser<org.ghrobotics.frc2019.auto.StartingPositions>()
    val autoModeChooser = enumSendableChooser<org.ghrobotics.frc2019.auto.AutoMode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("Main Display")

    private val globalXEntry: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Robot X", 0.0).entry
    private val globalYEntry: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Robot Y", 0.0).entry
    private val globalAEntry: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Robot Angle", 0.0).entry

    private val leftPositionEntry: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Left Encoder", 0.0).entry
    private val rightPositionEntry: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Right Encoder", 0.0).entry

    private val visionTargetX: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Vision Target X", 0.0).entry
    private val visionTargetY: NetworkTableEntry = org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Vision Target Y", 0.0).entry
    private val visionTargetRotation: NetworkTableEntry =
        org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Vision Target Rotation", 0.0).entry

    init {
        // Put choosers on dashboard
        org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Auto Mode",
            org.ghrobotics.frc2019.Network.autoModeChooser
        )
        org.ghrobotics.frc2019.Network.mainShuffleboardDisplay.add("Starting Position",
            org.ghrobotics.frc2019.Network.startingPositionChooser
        )
    }

    fun update() {
        org.ghrobotics.frc2019.Network.globalXEntry.setDouble(DriveSubsystem.localization().translation.x.feet)
        org.ghrobotics.frc2019.Network.globalYEntry.setDouble(DriveSubsystem.localization().translation.y.feet)
        org.ghrobotics.frc2019.Network.globalAEntry.setDouble(DriveSubsystem.localization().rotation.degree)

        org.ghrobotics.frc2019.Network.leftPositionEntry.setDouble(DriveSubsystem.leftMotor.getSelectedSensorPosition(0).toDouble())
        org.ghrobotics.frc2019.Network.rightPositionEntry.setDouble(DriveSubsystem.rightMotor.getSelectedSensorPosition(0).toDouble())

        val trackedObject = TargetTracker.bestTarget
        if (trackedObject != null) {
            val visionTargetPose = trackedObject.averagePose
            org.ghrobotics.frc2019.Network.visionTargetX.setDouble(visionTargetPose.translation.x.inch)
            org.ghrobotics.frc2019.Network.visionTargetY.setDouble(visionTargetPose.translation.y.inch)
            org.ghrobotics.frc2019.Network.visionTargetRotation.setDouble(visionTargetPose.rotation.degree)
        }
    }
}
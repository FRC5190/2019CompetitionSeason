/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.climb.ClimbSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.FalconRobot

object Robot : FalconRobot() {

    val emergencyReadySystems = ArrayList<EmergencyHandleable>()

    var emergencyActive = false

    // Initialize all systems.
    init {
        +DriveSubsystem
        +ElevatorSubsystem
        +ArmSubsystem
        +IntakeSubsystem
        +ClimbSubsystem

        Network
        Autonomous
        Trajectories

        VisionProcessing
    }

    override fun periodic() {
        Controls.update()
        Autonomous.update()
        LEDs.update()
/*
        val bestTargetRawData = RawDataTracker.bestTargetRawData
        if (bestTargetRawData != null) {
            DriveSubsystem.localization.addVisionSample(bestTargetRawData, Trajectories.kLoadingStation)
        }*/
    }

    override fun periodicNetwork() {
        Network.update()
    }

    override operator fun FalconSubsystem.unaryPlus() {
        addToSubsystemHandler(this)
        if (this is EmergencyHandleable) {
            emergencyReadySystems.add(this)
        }
    }
}

fun main() {
    FalconRobot.startRobot({ Robot }, 20.millisecond)
}
/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.Notifier
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.climb.ClimbSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.frc2019.vision.VisionProcessing
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.wrappers.FalconRobot

val kMainLoopDt = 20.millisecond

object Robot : FalconRobot() {

    val emergencyReadySystems = ArrayList<EmergencyHandleable>()

    var emergencyActive = false

    // Initialize all systems.
    init {
        +DriveSubsystem
        +ClimbSubsystem
        +ElevatorSubsystem
        +ArmSubsystem
        +IntakeSubsystem

        Network
        Autonomous

        CameraServer.getInstance().startAutomaticCapture()
            .setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15)
        VisionProcessing
        Notifier {
            TargetTracker.update()
        }.startPeriodic(.02)
    }

    override fun periodic() {
        Controls.update()
//        LEDs.update()
        Autonomous.update()
//        TargetTracker.update()
//        VisionProcessing.update()
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
    FalconRobot.startRobot({ Robot }, kMainLoopDt)
}
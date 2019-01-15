/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.RobotBase
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.robot.vision.VisionProcessing
import org.ghrobotics.lib.wrappers.FalconRobotBase

object Robot : FalconRobotBase(), CoroutineScope {

    override val coroutineContext = Job()

    // Initialize all systems.
    override fun initialize() {
        +DriveSubsystem

        org.ghrobotics.frc2019.Network
        org.ghrobotics.frc2019.auto.Autonomous
        org.ghrobotics.frc2019.auto.Trajectories

        VisionProcessing
    }

    override fun periodic() {
        org.ghrobotics.frc2019.Controls.update()
        org.ghrobotics.frc2019.Network.update()
        org.ghrobotics.frc2019.auto.Autonomous.update()
    }

    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { org.ghrobotics.frc2019.Robot }
    }
}
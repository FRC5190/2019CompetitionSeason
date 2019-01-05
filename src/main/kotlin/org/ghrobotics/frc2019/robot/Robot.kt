/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import edu.wpi.first.wpilibj.RobotBase
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.runBlocking
import org.ghrobotics.frc2019.robot.auto.Autonomous
import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.frc2019.robot.vision.startVisionProcessing
import org.ghrobotics.lib.wrappers.FalconRobotBase

object Robot : FalconRobotBase(), CoroutineScope {

    override val coroutineContext = Job()

    // Initialize all systems.
    override fun initialize() {
        +DriveSubsystem

        Network
        Autonomous
        Trajectories

        startVisionProcessing()
    }

    override fun periodic() {
        runBlocking {
            Controls.mainXbox.update()
        }
    }

    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { Robot }
    }
}
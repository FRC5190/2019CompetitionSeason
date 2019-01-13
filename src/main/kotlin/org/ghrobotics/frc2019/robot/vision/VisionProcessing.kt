package org.ghrobotics.frc2019.robot.vision

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.Robot
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

object VisionProcessing {
    val currentlyTrackedObject = DynamicObject(DriveSubsystem.localization)

    init {
        CameraServer.getInstance()
            .startAutomaticCapture()
            .apply {
                setPixelFormat(VideoMode.PixelFormat.kYUYV)
                setResolution(640, 480)
            }

        val visionDataChannel = Channel<VisionData>(Channel.CONFLATED)

        Robot.launch {
            JeVois(SerialPort.Port.kUSB1, visionDataChannel)

            for (visionData in visionDataChannel) {
                // Apply camera offset first
                val contourActualLocations =
                    visionData.targets.associateWith { Constants.kCenterToCamera + it.cameraRelativePose }
                // Find target thats closest
                val closestTarget = contourActualLocations.minBy { it.value.translation.norm.value } ?: continue

                // TODO we also need to get the angle OF the object (not angle to the object) for perfect tracking.
                currentlyTrackedObject.setSample(
                    visionData.timestamp,
                    closestTarget.value
                )
            }
        }
    }
}


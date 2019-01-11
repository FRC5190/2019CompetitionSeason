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
                setResolution(320, 252)
            }

        val visionDataChannel = Channel<VisionData>(Channel.CONFLATED)

        Robot.launch {
            createJeVois(SerialPort.Port.kUSB1, visionDataChannel)

            for (visionData in visionDataChannel) {
                if (visionData.targets.isNotEmpty()) {
                    val closestContour = visionData.targets.minBy { it.distance.value }!!

                    // TODO: we also need to get the angle OF the object (not angle to the object) for perfect tracking.
                    currentlyTrackedObject.setSample(
                        visionData.timestamp,
                        Constants.kCenterToCamera + closestContour.cameraRelativePose
                    )
                }
            }
        }
    }
}


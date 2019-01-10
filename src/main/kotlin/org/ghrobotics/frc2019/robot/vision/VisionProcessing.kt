package org.ghrobotics.frc2019.robot.vision

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.subsytems.drive.DriveSubsystem

fun CoroutineScope.startVisionProcessing() {
    CameraServer.getInstance()
        .startAutomaticCapture()
        .apply {
            setPixelFormat(VideoMode.PixelFormat.kYUYV)
            setResolution(320, 252)
        }

    val visionDataChannel = Channel<VisionData>()

    createJeVois(SerialPort.Port.kUSB1, visionDataChannel)

    launch {
        for (visionData in visionDataChannel) {
            if (visionData.contours.isNotEmpty()) {
                val closestContour = visionData.contours.minBy { it.distance.value }!!

                // TODO: we also need to get the angle OF the object (not angle to the object) for perfect tracking.
                VisionProcessing.currentlyTrackedObject.setSample(
                    visionData.timestamp,
                    Constants.kCenterToCamera + closestContour.cameraRelativePose
                )
            }
        }
    }
}

object VisionProcessing {
    val currentlyTrackedObject = DynamicObject(DriveSubsystem.localization)
}


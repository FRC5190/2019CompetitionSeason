package org.ghrobotics.frc2019.robot.vision

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch

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
            // Update DynamicObjects here
        }
    }
}


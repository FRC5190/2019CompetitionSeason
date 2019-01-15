package org.ghrobotics.frc2019.vision

import com.google.gson.JsonObject
import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.launch
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

object VisionProcessing {

    init {
        CameraServer.getInstance()
            .startAutomaticCapture()
            .apply {
                setPixelFormat(VideoMode.PixelFormat.kYUYV)
                setResolution(640, 480)
            }

        val visionDataChannel = Channel<VisionData>(Channel.CONFLATED)

        org.ghrobotics.frc2019.Robot.launch {
            JeVois(SerialPort.Port.kUSB1, visionDataChannel)

            for (visionData in visionDataChannel) {
                val robotPose = DriveSubsystem.localization[visionData.timestamp]

                TargetTracker.addSamples(
                    visionData.timestamp,
                    visionData.targets
                        .asSequence()
                        .mapNotNull(::processReflectiveTape)
                        .map { robotPose + it }
                        .toList()
                )
            }
        }
    }

    private fun processReflectiveTape(data: JsonObject): Pose2d? {
        // {"angleH": -8.53125, "angleV": 13.031250000000002, "distance": 78.51851851851852}

        val angle = data["angle"].asDouble.degree
        val rotation = data["rotation"].asDouble.degree
        val distance = data["distance"].asDouble.inch

        return org.ghrobotics.frc2019.Constants.kCenterToCamera + Pose2d(Translation2d(distance, angle), rotation)
    }

//    private fun processWhiteTape(data: JsonObject): Pose2d? {
//        // {"one": {"h": -16.875, "v": -32.4375}, "two": {"h": 60.0, "v": -32.4375}}
//
//        val oneInFrameOfCamera = processWhiteTapePoint(data["one"].asJsonObject)
//        val twoInFrameOfCamera = processWhiteTapePoint(data["two"].asJsonObject)
//
//        if (oneInFrameOfCamera.distance(twoInFrameOfCamera) < Constants.kMinLineLength.value) {
//            return null // Ignore small lines
//        }
//
//        return Constants.kCenterToCamera + Pose2d(
//            (oneInFrameOfCamera + twoInFrameOfCamera) / 2.0,
//            Math.atan2(
//                oneInFrameOfCamera.x.value - twoInFrameOfCamera.x.value,
//                oneInFrameOfCamera.y.value - twoInFrameOfCamera.y.value
//            ).degree
//        )
//    }
//
//    private fun processWhiteTapePoint(data: JsonObject): Translation2d {
//        // {"h": -16.875, "v": -32.4375}
//
//        val h = data["h"].asDouble
//        val v = Constants.kCameraYaw + data["v"].asDouble
//
//        val xDistance = (Constants.kGroundToCamera / Math.tan(Math.toRadians(v))).absoluteValue
//        val yDistance = xDistance * Math.tan(Math.toRadians(h))
//
//        return Translation2d(xDistance, yDistance)
//    }

}


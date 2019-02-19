package org.ghrobotics.frc2019.vision

import com.fazecast.jSerialComm.SerialPort
import com.google.gson.JsonObject
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

object VisionProcessing {

    init {
        println(SerialPort.getCommPorts().joinToString(separator = "\n") { it.descriptivePortName })
//        JeVois(SerialPort.Port.kUSB1) { visionData ->
//            val robotPose = DriveSubsystem.localization[visionData.timestamp]
//
//            TargetTracker.addSamples(
//                visionData.timestamp,
//                visionData.targets
//                    .asSequence()
//                    .mapNotNull { processReflectiveTape(it, Constants.kCenterToFrontCamera) }
//                    .map { robotPose + it }.toList()
//            )
//        }
//        JeVois(SerialPort.Port.kUSB1) { visionData ->
//            val robotPose = DriveSubsystem.localization[visionData.timestamp]
//
//            TargetTracker.addSamples(
//                visionData.timestamp,
//                visionData.targets
//                    .asSequence()
//                    .mapNotNull { processReflectiveTape(it, Constants.kCenterToBackCamera) }
//                    .map { robotPose + it }.toList()
//            )
//        }
    }

    private fun processReflectiveTape(data: JsonObject, transform: Pose2d): Pose2d? {
        val angle = data["angle"].asDouble.degree
        val rotation = -data["rotation"].asDouble.degree + angle + 180.degree
        val distance = data["distance"].asDouble.inch

//        println("${distance.inch}, ${angle.degree}")

        return transform + Pose2d(Translation2d(distance, angle), rotation)
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


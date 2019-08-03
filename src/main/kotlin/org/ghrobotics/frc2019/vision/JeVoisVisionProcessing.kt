package org.ghrobotics.frc2019.vision

import com.google.gson.JsonObject
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import kotlin.math.absoluteValue

object JeVoisVisionProcessing {

    fun processData(visionData: VisionData) {

        if (visionData.isFront && ElevatorSubsystem.position in Constants.kElevatorBlockingCameraRange) {
            return
        }

        val robotPose = DriveSubsystem.localization[visionData.timestamp.second]

        TargetTracker.addSamples(
            visionData.timestamp,
            visionData.targets
                .asSequence()
                .mapNotNull {
                    processReflectiveTape(
                        it,
                        if (visionData.isFront) Constants.kCenterToFrontCamera else Constants.kCenterToBackCamera
                    )
                }
                .filter {
                    // We cannot be the vision target :)
                    it.translation.x.absoluteValue > Constants.kRobotLength.value / 2.0
                        || it.translation.y.absoluteValue > Constants.kRobotWidth.value / 2.0
                }
                .map { robotPose + it }.toList()
        )
    }

    private fun processReflectiveTape(data: JsonObject, transform: Pose2d): Pose2d? {
        val angle = data["angle"].asDouble.degree
        val rotation = -data["rotation"].asDouble.degree + angle + 180.degree
        val distance = data["distance"].asDouble.inch

        return transform + Pose2d(Translation2d(distance, angle), rotation)
    }

}


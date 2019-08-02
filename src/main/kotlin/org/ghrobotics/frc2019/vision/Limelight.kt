@file:Suppress("PrivatePropertyName", "LocalVariableName")

package org.ghrobotics.frc2019.vision

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.wrappers.networktables.get
import kotlin.math.tan

class Limelight(
    private val limelight_height: Length,
    limelight_angle: Rotation2d,
    private val target_height: Length
) {
    private val limelight_table_: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")
    private val limelight_angle_ = limelight_angle.value

    private val tx_: NetworkTableEntry = limelight_table_["tx"]
    private val ty_: NetworkTableEntry = limelight_table_["ty"]
    private val tl_: NetworkTableEntry = limelight_table_["tl"]
    private val tv_: NetworkTableEntry = limelight_table_["tv"]

    init {
        turnOffLED()
    }

    var isAlive: Boolean = false
        private set

    fun turnOnLED() {
        limelight_table_["ledMode"].setNumber(3)
    }

    fun turnOffLED() {
        limelight_table_["ledMode"].setNumber(1)
    }

    fun blinkLEDs() {
        limelight_table_["ledMode"].setNumber(2)
    }

    fun updateTargetTracker() {
        val latency: Double = tl_.getDouble(0.0) + 11
        isAlive = latency > 11

        if (tv_.getNumber(0.0) == 0.0) return

        // tx has a negative sign because the Limelight is CW positive. We don't want that.
        val tx: Double = -Math.toRadians(tx_.getDouble(0.0))
        val ty: Double = +Math.toRadians(ty_.getDouble(0.0))

        val distance_to_target = (target_height - limelight_height) / tan(limelight_angle_ + ty)
        if (distance_to_target < Constants.kRobotLength / 2.2) return

        val timestamp: Double = Timer.getFPGATimestamp() - latency / 1000.0
        val transform = Translation2d(distance_to_target, Rotation2d(tx))
        val drive_location = DriveSubsystem.localization[timestamp.second]

        TargetTracker.addSamples(
            timestamp, listOfNotNull(
                drive_location + (Constants.kCenterToFrontCamera + Pose2d(transform))
            )
        )
    }
}
/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.subsytems.drive

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.common.Constants
import org.ghrobotics.frc2019.common.subsystems.drive.DriveSubsystemBase
import org.ghrobotics.frc2019.robot.Robot
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import kotlin.properties.Delegates.observable

object DriveSubsystem : DriveSubsystemBase, TankDriveSubsystem() {

    // Gearboxes
    private val leftGearbox = DriveGearbox(
        Constants.kLeftMasterId,
        Constants.kLeftSlaveId1,
        Constants.kLeftSlaveId2,
        false
    )
    private val rightGearbox = DriveGearbox(
        Constants.kRightMasterId,
        Constants.kRightSlaveId1,
        Constants.kRightSlaveId2,
        true
    )

    // Master motors
    override val leftMotor get() = leftGearbox.master
    override val rightMotor get() = rightGearbox.master

    private val allMasters get() = listOf(leftMotor, rightMotor)

    // Shifter for two-speed gearbox
    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    // Type of localization to determine position on the field
    override val localization = TankEncoderLocalization(
        PigeonIMU(Constants.kPigeonIMUId).asSource(), { leftMotor.sensorPosition }, { rightMotor.sensorPosition },
        Robot.coroutineContext
    )

    // Shift up and down
    var lowGear by observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.set(true)
        } else {
            shifter.set(false)
        }
    }

    init {
        lowGear = false
        defaultCommand = ManualDriveCommand()
        allMasters.forEach { it.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms) }
    }
}

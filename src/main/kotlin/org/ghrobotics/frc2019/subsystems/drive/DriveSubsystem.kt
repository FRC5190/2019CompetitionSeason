/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import com.ctre.phoenix.sensors.PigeonIMU
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import kotlin.properties.Delegates.observable

object DriveSubsystem : TankDriveSubsystem(), EmergencyHandleable {

    // Gearboxes
    private val leftGearbox = DriveGearbox(
        Constants.kDriveLeftMasterId,
        Constants.kDriveLeftSlaveId,
        false
    )
    private val rightGearbox = DriveGearbox(
        Constants.kDriveRightMasterId,
        Constants.kDriveRightSlaveId,
        true
    )

    // Master motors
    override val leftMotor get() = leftGearbox.master
    override val rightMotor get() = rightGearbox.master

    // Shifter for two-speed gearbox
    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    // Type of localization to determine position on the field
    override val localization = FusedLocalization(
        PigeonIMU(Constants.kPigeonIMUId).run {
            setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10)
            return@run { fusedHeading.degree }
        },
        leftMotor::sensorPosition,
        rightMotor::sensorPosition
    )

    // Shift up and down
    var lowGear by observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.set(true)
        } else {
            shifter.set(false)
        }
    }

    val kPathFollowingDt = 10.millisecond

    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    init {
        lowGear = false
        defaultCommand = ManualDriveCommand()
    }

    override fun activateEmergency() {
        leftGearbox.setClosedLoopGains()
        rightGearbox.setClosedLoopGains()
    }

    override fun recoverFromEmergency() {
        leftGearbox.zeroClosedLoopGains()
        rightGearbox.zeroClosedLoopGains()
    }
}
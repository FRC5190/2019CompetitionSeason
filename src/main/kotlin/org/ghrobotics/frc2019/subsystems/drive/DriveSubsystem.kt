/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import kotlin.properties.Delegates.observable

object DriveSubsystem : TankDriveSubsystem(), EmergencyHandleable {

    // Gearboxes
    private val leftGearbox = DriveGearbox(Constants.kDriveLeftMasterId, Constants.kDriveLeftSlaveId, false)
    private val rightGearbox = DriveGearbox(Constants.kDriveRightMasterId, Constants.kDriveRightSlaveId, true)

    // Shaft encoders
//    val lEncoder = Encoder(Constants.kLeftDriveEncoderA, Constants.kLeftDriveEncoderB, true)
//    val rEncoder = Encoder(Constants.kRightDriveEncoderA, Constants.kRightDriveEncoderB, false)

    // Master motors
    override val leftMotor get() = leftGearbox.master
    override val rightMotor get() = rightGearbox.master

    // Autonomous path navigation
    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    // Velocity of the drivetrain at a given point
    val velocity get() = (leftMotor.velocity + rightMotor.velocity) / 2.0

    // Shifter for two-speed gearbox
    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    // Type of localization to determine position on the field
    override val localization = TankEncoderLocalization(
        IntakeSubsystem.pigeonSource,
        { leftMotor.sensorPosition.value },
        { rightMotor.sensorPosition.value }
    )

//    override val localization = TankEncoderLocalization(
//        pigeon.asSource(),
//        { lEncoder.distance.meter },
//        { rEncoder.distance.meter }
//    )

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

//        listOf(lEncoder, rEncoder).forEach {
//            it.distancePerPulse = 3.353 / 9996.5 * 4
//        }
    }


//    const val kP = 4.0 / 12.0
//
//    override fun setOutput(output: TrajectoryTrackerOutput) {
//        setOutputFromKinematics(output.differentialDriveVelocity)
//    }
//
//    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
//        val leftDesired = wheelVelocities.left * Constants.kDriveWheelRadius.value
//        val rightDesired = wheelVelocities.right * Constants.kDriveWheelRadius.value
//
//        val leftCurrent = leftMotor.velocity.value
//        val rightCurrent = rightMotor.velocity.value
//
//        val lError = (leftDesired - leftCurrent)
//        val rError = (rightDesired - rightCurrent)
//
//        val (l, r) = lError * kP to rError * kP
//        leftMotor.percentOutput = l + wheelVoltages.left / 12.0
//        rightMotor.percentOutput = r + wheelVoltages.right / 12.0
//    }

    fun notWithinRegion(region: Rectangle2d) =
        ConditionCommand { !region.contains(robotPosition.translation) }

    override fun activateEmergency() {
        zeroOutputs()
        leftGearbox.zeroClosedLoopGains()
        rightGearbox.zeroClosedLoopGains()
    }

    override fun recoverFromEmergency() {
        leftGearbox.setClosedLoopGains()
        rightGearbox.setClosedLoopGains()
    }
}
/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.Trajectories
import org.ghrobotics.frc2019.auto.VisionAssistedTrajectory
import org.ghrobotics.frc2019.vision.DynamicObject
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.kEpsilon
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.Source
import kotlin.properties.Delegates.observable

object DriveSubsystem : TankDriveSubsystem() {

    /* HARDWARE */

    // Gearboxes
    private val leftGearbox = DriveGearbox(
        Constants.kLeftMasterId,
        Constants.kLeftSlaveId1,
        false
    )
    private val rightGearbox = DriveGearbox(
        Constants.kRightMasterId,
        Constants.kRightSlaveId1,
        true
    )

    // Master motors
    override val leftMotor get() = leftGearbox.master
    override val rightMotor get() = rightGearbox.master

    private val allMasters
        get() = listOf(
            leftMotor,
            rightMotor
        )

    // Shifter for two-speed gearbox
    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    // Type of localization to determine position on the field
    override val localization = TankEncoderLocalization(
        PigeonIMU(Constants.kPigeonIMUId).asSource(),
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


    /* SOFTWARE */

    val distanceTraveled
        get() = (leftMotor.sensorPosition + rightMotor.sensorPosition) / 2.0

    val velocity
        get() = (leftMotor.velocity + rightMotor.velocity) / 2.0

    val kPathFollowingDt = 10.millisecond

    override val differentialDrive = Trajectories.differentialDrive
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    init {
        lowGear = false
        defaultCommand = ManualDriveCommand()
    }

    fun followVisionAssistedTrajectory(
        trajectory: TimedTrajectory<Pose2dWithCurvature>,
        mirrored: Source<Boolean>,
        dynamicObject: DynamicObject,
        expectedLocation: Translation2d,
        dt: Time = kPathFollowingDt
    ) = followTrajectory(
        trajectory = VisionAssistedTrajectory(
            originalTrajectory = if (!mirrored()) trajectory else trajectory.mirror(),
            dynamicObject = dynamicObject,
            expectedTargetLocation = expectedLocation
        ),
        pathMirrored = false,
        dt = dt
    )

    override fun autoReset() {
        allMasters.forEach {
            it.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)
            it.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms)
        }
    }

    override fun teleopReset() {
        allMasters.forEach {
            it.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms)
            it.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100)
        }
    }

    override fun zeroOutputs() {
        leftMotor.velocity = 0.meter.velocity
        rightMotor.velocity = 0.meter.velocity
    }
}

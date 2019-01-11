/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot.subsytems.drive

import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.robot.Constants
import org.ghrobotics.frc2019.robot.auto.Trajectories
import org.ghrobotics.frc2019.robot.vision.DynamicObject
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
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

    private val allMasters get() = listOf(leftMotor, rightMotor)

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

    val kPathFollowingDt = 10.millisecond

    override val differentialDrive = Trajectories.differentialDrive
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    init {
        lowGear = false
        defaultCommand = ManualDriveCommand()
    }

    fun driveToLocation(location: Source<Pose2d>): FalconCommand {
        return followTrajectory(
            trajectory = { DriveOTFSplineGenerator.create(location()) },
            pathMirrored = false,
            dt = kPathFollowingDt
        )
    }

    fun followVisionAssistedTrajectory(
        trajectory: Source<TimedTrajectory<Pose2dWithCurvature>>,
        mirrored: Source<Boolean>,
        dynamicObject: Source<DynamicObject>,
        expectedLocation: Source<Translation2d>,
        error: Source<Length> = { 2.feet },
        dt: Time = kPathFollowingDt
    ) = VisionAssistedTrajectoryTrackerCommand(
        trajectory.map {
            if (mirrored()) it.mirror() else it
        },
        dynamicObject, expectedLocation, error, dt
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

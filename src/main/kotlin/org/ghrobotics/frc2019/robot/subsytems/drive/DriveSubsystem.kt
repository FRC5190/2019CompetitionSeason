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
import org.ghrobotics.frc2019.robot.auto.generateTrajectory
import org.ghrobotics.frc2019.robot.auto.waypoints
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.statespace.*
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
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

    private val velocityReferenceTracker = StateSpaceLoop(
        StateSpacePlantCoefficients(
            DriveSSMatrices.A,
            DriveSSMatrices.A.inv(),
            DriveSSMatrices.B,
            DriveSSMatrices.C,
            DriveSSMatrices.D
        ),
        StateSpaceControllerCoefficients(DriveSSMatrices.K, DriveSSMatrices.Kff),
        StateSpaceObserverCoefficients(DriveSSMatrices.L)
    )


    init {
        lowGear = false
        defaultCommand = ManualDriveCommand()
        allMasters.forEach { it.kP = 0.75; it.kD = 1.0 }
    }

    fun driveToLocation(location: Pose2d) = driveToLocation { location }

    fun driveToLocation(location: Source<Pose2d>): FalconCommand {
        return followTrajectory(
            trajectory = { waypoints(localization(), location()).generateTrajectory(false) },
            pathMirrored = false,
            dt = kPathFollowingDt
        )
    }

    override fun autoReset() {
        allMasters.forEach {
            it.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)
            it.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms)
        }
    }

    override fun teleopReset() {
        allMasters.forEach { it.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms) }
        allMasters.forEach { it.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100) }
    }

    override fun zeroOutputs() {
        leftMotor.velocity = 0.meter.velocity
        rightMotor.velocity = 0.meter.velocity
    }


    // Remove this to use the Talon SRX Velocity Loop with Arbitrary Feedforward
//    override fun setOutput(
//        wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState
//    ) {
//        val r = Matrix(
//            arrayOf(
//                doubleArrayOf(wheelVelocities.left * Constants.kWheelRadius.value),
//                doubleArrayOf(wheelVelocities.right * Constants.kWheelRadius.value)
//            )
//        )
//
//        val y = Matrix(
//            arrayOf(
//                doubleArrayOf(leftMotor.sensorVelocity.value),
//                doubleArrayOf(rightMotor.sensorVelocity.value)
//            )
//        )
//
//        velocityReferenceTracker.nextR = r
//        velocityReferenceTracker.correct(y)
//        val u = velocityReferenceTracker.update()
//
//        leftMotor.percentOutput = u.data[0][0] / 12.0
//        rightMotor.percentOutput = u.data[1][0] / 12.0
//    }
}

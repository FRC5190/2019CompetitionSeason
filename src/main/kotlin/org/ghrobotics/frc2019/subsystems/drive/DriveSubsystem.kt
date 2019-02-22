/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems.drive

import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.kMainLoopDt
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.climb.ClimbSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.statespace.*
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.sensors.asSource
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
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
    private val pigeon = PigeonIMU(Constants.kPigeonIMUId)

    // Type of localization to determine position on the field
    override val localization = TankEncoderLocalization(
        pigeon.asSource(),
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
        pigeon.setTemperatureCompensationDisable(true)
    }

    fun followVisionAssistedTrajectory(
        trajectory: TimedTrajectory<Pose2dWithCurvature>,
        pathMirrored: Source<Boolean>,
        or: Length,
        ir: Length
    ) = VisionAssistedTrajectoryTrackerCommand(pathMirrored.map(trajectory.mirror(), trajectory), or, ir)

    /*override fun setOutput(
        wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState
    ) {
        System.out.printf(
            "L Reference: %3.3f, R Reference: %3.3f, L Real: %3.3f, R Real: %3.3f, " +
                "L Voltage: %3.3f, R Voltage: %3.3f%n",
            wheelVelocities.left * differentialDrive.wheelRadius,
            wheelVelocities.right * differentialDrive.wheelRadius,
            leftMotor.velocity.value, rightMotor.velocity.value,
            leftMotor.voltageOutput.value, rightMotor.voltageOutput.value
        )
        val r = Matrix(
            arrayOf(
                doubleArrayOf(wheelVelocities.left * Constants.kDriveWheelRadius.value),
                doubleArrayOf(wheelVelocities.right * Constants.kDriveWheelRadius.value)
            )
        )

        val y = Matrix(
            arrayOf(
                doubleArrayOf(leftMotor.sensorVelocity.value),
                doubleArrayOf(rightMotor.sensorVelocity.value)
            )
        )

        velocityReferenceTracker.nextR = r
        velocityReferenceTracker.correct(y)
        val u = velocityReferenceTracker.update()

        leftMotor.percentOutput = u.data[0][0] / 12.0
        rightMotor.percentOutput = u.data[1][0] / 12.0
    }*/

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
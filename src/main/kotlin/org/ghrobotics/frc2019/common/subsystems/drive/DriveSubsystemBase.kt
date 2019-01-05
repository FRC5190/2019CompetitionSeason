package org.ghrobotics.frc2019.common.subsystems.drive

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.common.Constants
import org.ghrobotics.lib.mathematics.statespace.*
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase
import kotlin.math.pow

interface DriveSubsystemBase : DifferentialTrackerDriveBase {

    private val velocityReferenceTracker get() = _velocityReferenceTracker

    private val dcTransmission get() = _dcTransmission

    // Differential drive model that represents the drivetrain
    override val differentialDrive get() = _differentialDrive

    override val trajectoryTracker get() = _trajectoryTracker

    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
        val r = Matrix(
            arrayOf(
                doubleArrayOf(wheelVelocities.left * Constants.kWheelRadius.value),
                doubleArrayOf(wheelVelocities.right * Constants.kWheelRadius.value)
            )
        )
        val y = Matrix(
            arrayOf(
                doubleArrayOf(leftMotor.velocity.value),
                doubleArrayOf(rightMotor.velocity.value)
            )
        )

        velocityReferenceTracker.nextR = r
        velocityReferenceTracker.correct(y)
        val u = velocityReferenceTracker.update()

        leftMotor.percentOutput = u.data[0][0] / 12.0
        rightMotor.percentOutput = u.data[1][0] / 12.0
    }

    @Suppress("ObjectPropertyName", "ObjectPropertyNaming")
    private companion object {
        val _velocityReferenceTracker = StateSpaceLoop(
            StateSpacePlantCoefficients(
                DriveSSMatrices.A,
                DriveSSMatrices.A.inv(),
                DriveSSMatrices.B,
                DriveSSMatrices.C,
                DriveSSMatrices.D
            ),
            StateSpaceControllerCoefficients(
                DriveSSMatrices.K, DriveSSMatrices.Kff
            ),
            StateSpaceObserverCoefficients(DriveSSMatrices.L)
        )

        val _dcTransmission = DCMotorTransmission(
            1 / Constants.kVDrive,
            Constants.kWheelRadius.value.pow(2) * Constants.kRobotMass / (2.0 * Constants.kADrive),
            Constants.kStaticFrictionVoltage
        )

        // Differential drive model that represents the drivetrain
        val _differentialDrive = DifferentialDrive(
            Constants.kRobotMass,
            Constants.kRobotMomentOfInertia,
            Constants.kRobotAngularDrag,
            Constants.kWheelRadius.value,
            Constants.kTrackWidth.value / 2.0,
            _dcTransmission,
            _dcTransmission
        )

        val _trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)
    }
}
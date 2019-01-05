package org.ghrobotics.frc2019.sim.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.common.Constants
import org.ghrobotics.frc2019.common.subsystems.drive.DriveSSMatrices
import org.ghrobotics.frc2019.common.subsystems.drive.DriveSubsystemBase
import org.ghrobotics.frc2019.sim.motors.SimulatedMotor
import org.ghrobotics.lib.mathematics.statespace.control.Matrix
import org.ghrobotics.lib.mathematics.statespace.control.plus
import org.ghrobotics.lib.mathematics.statespace.control.times
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Twist2d
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.radian

object SimulatedDriveSubsystem : DriveSubsystemBase {

    var x = Matrix(2, 1)
        private set

    var y = Matrix(2, 1)
        private set

    override var robotLocation = Pose2d()

    override val leftMotor = SimulatedMotor(0.meter)
    override val rightMotor = SimulatedMotor(0.meter)

    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
        super.setOutput(wheelVelocities, wheelVoltages)

        val u = Matrix(
            arrayOf(
                doubleArrayOf(leftMotor.voltageOutput.value), doubleArrayOf(rightMotor.voltageOutput.value)
            )
        )

        x = DriveSSMatrices.A * x + DriveSSMatrices.B * u
        y = DriveSSMatrices.C * x + DriveSSMatrices.D * u

        leftMotor.velocity = x.data[0][0].meter.velocity
        rightMotor.velocity = x.data[1][0].meter.velocity

        val vc = (x.data[0][0] + x.data[1][0]) / 2.0
        val wc = (x.data[1][0] - x.data[0][0]) / Constants.kTrackWidth.value

        robotLocation += (Twist2d(vc.meter, 0.meter, wc.radian) * 0.005).asPose
    }
}
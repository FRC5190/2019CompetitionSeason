package org.ghrobotics.frc2019.subsystems.drive

import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.Trajectories
import org.junit.Test

class DriveModelTest {
    @Test
    fun testAccelerationCurve() {
        val voltage = DifferentialDrive.WheelState(12.0, 12.0)
        var velocity = DifferentialDrive.WheelState(0.0, 0.0)
        val dt = 0.01

        var t = 0.0

        val time = ArrayList<Double>()
        val speed = ArrayList<Double>()

        for (i in 0..500) {
            speed.add(velocity.left * Constants.kWheelRadius.value)
            time.add(t)

            val predictedAcceleration =
                Trajectories.differentialDrive.solveForwardDynamics(velocity, voltage).wheelAcceleration
            velocity = DifferentialDrive.WheelState(
                velocity.left + predictedAcceleration.left * dt,
                velocity.right + predictedAcceleration.right * dt
            )

            t += dt
        }

        println("Max Speed: ${speed.max()}")

//        SwingWrapper(
//            QuickChart.getChart(
//                "Speed with Constant Voltage",
//                "T",
//                "V",
//                "Plot",
//                time.toDoubleArray(),
//                speed.toDoubleArray()
//            )
//        ).displayChart()
//        Thread.sleep(100000)
    }
}
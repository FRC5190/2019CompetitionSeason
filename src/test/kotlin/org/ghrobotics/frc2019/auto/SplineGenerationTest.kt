package org.ghrobotics.frc2019.auto

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricQuinticHermiteSpline
import org.ghrobotics.lib.mathematics.twodim.polynomials.ParametricSplineGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.DistanceTrajectory
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.nanosecond
import org.junit.Test
import kotlin.system.measureNanoTime

class SplineGenerationTest {

    private val points = listOf(Pose2d(0.meter, 0.meter, 0.degree), Pose2d(2.meter, 3.meter, 120.degree))

    @Test
    fun testSplineGenerationTime() {
        val time = measureNanoTime {
            DistanceTrajectory(
                ParametricSplineGenerator.parameterizeSplines(
                    listOf(
                        ParametricQuinticHermiteSpline(
                            points[0],
                            points[1]
                        )
                    ),
                    2.0, .25, 5.degree
                )
            )
        }
        println("Generation of Distance Trajectory took ${time.nanosecond.millisecond} milliseconds.")
    }

}
package org.ghrobotics.frc2019.subsystems.drive

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class SlopeNativeUnitModelTest {

    private val slopeNativeUnitModel = SlopeNativeUnitModel(
        1.5700931336.feet,
        1440.STU
    )

    @Test
    fun testToPosition() {
        val one = 720.STU
        val two = 2160.STU

        val onePosition = slopeNativeUnitModel.fromNativeUnitPosition(one)
        val twoPosition = slopeNativeUnitModel.fromNativeUnitPosition(two)

        assert(onePosition.feet epsilonEquals 0.7850465668)
        assert(twoPosition.feet epsilonEquals 2.3551397004)
    }

    @Test
    fun testFromPosition() {
        val one = 0.7850465668.feet
        val two = 2.3551397004.feet

        val oneNativeUnits = slopeNativeUnitModel.toNativeUnitPosition(one)
        val twoNativeUnits = slopeNativeUnitModel.toNativeUnitPosition(two)

        assert(oneNativeUnits.value epsilonEquals 720.0)
        assert(twoNativeUnits.value epsilonEquals 2160.0)
    }

    @Test
    fun testWheelRadius() {
        val radius = slopeNativeUnitModel.wheelRadius(1440.STU)

        assert(radius.inch epsilonEquals 2.998657)
    }

}
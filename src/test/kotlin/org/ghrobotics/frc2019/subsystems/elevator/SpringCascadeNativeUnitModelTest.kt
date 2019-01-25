package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.derivedunits.inchesPerSecond
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class SpringCascadeNativeUnitModelTest {

    private val model = SpringCascadeNativeUnitModel(
        switchHeight = 8.inch,
        switchNativeUnit = 2310.STU,
        afterSwitchHeightSample = 40.inch,
        afterSwitchNativeUnitSample = 6930.STU
    )

    @Test
    fun testToNativeUnit() {
        val one = 5.inch
        val two = 40.inch
        val three = 30.inch

        val oneNativeUnits = model.toNativeUnitPosition(one)
        val twoNativeUnits = model.toNativeUnitPosition(two)
        val threeNativeUnits = model.toNativeUnitPosition(three)

        assert(oneNativeUnits.value epsilonEquals 1443.75)
        assert(twoNativeUnits.value epsilonEquals 6930.0)
        assert(threeNativeUnits.value epsilonEquals 5486.25)
    }

    @Test
    fun testFromNativeUnit() {
        val one = 1443.75.STU
        val two = 6930.STU
        val three = 5486.25.STU

        val oneLength = model.fromNativeUnitPosition(one)
        val twoLength = model.fromNativeUnitPosition(two)
        val threeLength = model.fromNativeUnitPosition(three)

        assert(oneLength.inch epsilonEquals 5.0)
        assert(twoLength.inch epsilonEquals 40.0)
        assert(threeLength.inch epsilonEquals 30.0)
    }

    @Test
    fun testToNativeUnitVelocity() {
        val one = 5.inch.velocity
        val two = 40.inch.velocity
        val three = 30.inch.velocity

        val oneNativeUnits = model.toNativeUnitVelocity(one)
        val twoNativeUnits = model.toNativeUnitVelocity(two)
        val threeNativeUnits = model.toNativeUnitVelocity(three)

        assert(oneNativeUnits.value epsilonEquals 721.875)
        assert(twoNativeUnits.value epsilonEquals 5775.0)
        assert(threeNativeUnits.value epsilonEquals 4331.25)
    }

    @Test
    fun testFromNativeUnitVelocity() {
        val one = 721.875.STU.velocity
        val two = 5775.STU.velocity
        val three = 4331.25.STU.velocity

        val oneLength = model.fromNativeUnitVelocity(one)
        val twoLength = model.fromNativeUnitVelocity(two)
        val threeLength = model.fromNativeUnitVelocity(three)

        assert(oneLength.inchesPerSecond epsilonEquals 5.0)
        assert(twoLength.inchesPerSecond epsilonEquals 40.0)
        assert(threeLength.inchesPerSecond epsilonEquals 30.0)
    }

}
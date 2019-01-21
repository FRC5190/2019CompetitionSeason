package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class SpringCascadingNativeUnitModelTest {

    private val model = SpringCascadingNativeUnitModel(
        switchHeight = 8.inch,
        switchNativeUnit = 2310.STU,
        afterSwitchHeightSample = 40.inch,
        afterSwitchNativeUnitSample = 6930.STU
    )

    @Test
    fun testFromModel() {
        val one = 5.inch
        val two = 40.inch
        val three = 30.inch

        val oneNativeUnits = model.fromModel(one)
        val twoNativeUnits = model.fromModel(two)
        val threeNativeUnits = model.fromModel(three)

        assert(oneNativeUnits.value epsilonEquals 1443.75)
        assert(twoNativeUnits.value epsilonEquals 6930.0)
        assert(threeNativeUnits.value epsilonEquals 5486.25)
    }

    @Test
    fun testToModel() {
        val one = 1443.75.STU
        val two = 6930.STU
        val three = 5486.25.STU

        val oneLength = model.toModel(one)
        val twoLength = model.toModel(two)
        val threeLength = model.toModel(three)

        assert(oneLength.inch epsilonEquals 5.0)
        assert(twoLength.inch epsilonEquals 40.0)
        assert(threeLength.inch epsilonEquals 30.0)
    }

    @Test
    fun testFromOverallModel() {
        val one = 5.inch
        val two = 40.inch
        val three = 30.inch

        val oneNativeUnits = model.fromOverallModel(one.value).STU
        val twoNativeUnits = model.fromOverallModel(two.value).STU
        val threeNativeUnits = model.fromOverallModel(three.value).STU

        assert(oneNativeUnits.value epsilonEquals 721.875)
        assert(twoNativeUnits.value epsilonEquals 5775.0)
        assert(threeNativeUnits.value epsilonEquals 4331.25)
    }

    @Test
    fun testToOverallModel() {
        val one = 721.875.STU
        val two = 5775.STU
        val three = 4331.25.STU

        val oneLength = model.toOverallModel(one.value).meter
        val twoLength = model.toOverallModel(two.value).meter
        val threeLength = model.toOverallModel(three.value).meter

        assert(oneLength.inch epsilonEquals 5.0)
        assert(twoLength.inch epsilonEquals 40.0)
        assert(threeLength.inch epsilonEquals 30.0)
    }

}
package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.junit.Test

class CascadingElevatorTest {

    @Test
    fun testElevatorHelperBeforeSwitch(){
        val beforeSwitchHeight = 5.inch
        val beforeSwitchSTU = CascadingElevatorHelper.getNativeUnitsFromHeight(beforeSwitchHeight)
        assert(beforeSwitchSTU.value epsilonEquals 1443.75)
        val beforeSwitchHeightConvertedBack = CascadingElevatorHelper.getHeightFromNativeUnit(beforeSwitchSTU)
        assert(beforeSwitchHeight epsilonEquals beforeSwitchHeightConvertedBack)
    }

    @Test
    fun testElevatorHelperAfterSwitch(){
        val afterSwitchHeight = 40.inch
        val afterSwitchSTU = CascadingElevatorHelper.getNativeUnitsFromHeight(afterSwitchHeight)
        assert(afterSwitchSTU.value epsilonEquals 6930.0)
        val afterSwitchHeightConvertedBack = CascadingElevatorHelper.getHeightFromNativeUnit(afterSwitchSTU)
        assert(afterSwitchHeight epsilonEquals afterSwitchHeightConvertedBack)
    }

    @Test
    fun test() {
        println(CascadingElevatorHelper.getNativeUnitsFromHeight(74.inch).velocity.STUPer100ms)
    }

}
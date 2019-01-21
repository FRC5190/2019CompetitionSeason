package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.STU

class SpringCascadingNativeUnitModel(
    switchHeight: Length,
    switchNativeUnit: NativeUnit,
    afterSwitchHeightSample: Length,
    afterSwitchNativeUnitSample: NativeUnit
) : NativeUnitModel<Length>(0.STU) {

    private val switchHeight = switchHeight.value
    private val switchNativeUnit = switchNativeUnit.value
    private val beforeSwitchSlope = switchNativeUnit.value / switchHeight.value
    private val afterSwitchSlope = (afterSwitchNativeUnitSample.value - switchNativeUnit.value) /
        (afterSwitchHeightSample.value - switchHeight.value)

    override fun createNew(newValue: Double) = Length(newValue)

    override fun toModel(value: Double): Double =
        when {
            value < switchNativeUnit -> value / beforeSwitchSlope
            else -> {
                val afterSwitchNativeUnits = value - switchNativeUnit
                val afterSwitchHeight = afterSwitchNativeUnits / afterSwitchSlope
                switchHeight + afterSwitchHeight
            }
        }

    override fun fromModel(value: Double): Double =
        when {
            value < switchHeight -> value * beforeSwitchSlope
            else -> {
                val afterSwitchHeight = value - switchHeight
                val afterSwitchNativeUnits = afterSwitchHeight * afterSwitchSlope
                switchNativeUnit + afterSwitchNativeUnits
            }
        }

    fun toOverallModel(value: Double) = value / afterSwitchSlope
    fun fromOverallModel(value: Double) = value * afterSwitchSlope

}
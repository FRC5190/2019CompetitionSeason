package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel

class SpringCascadeNativeUnitModel(
    switchHeight: Length,
    switchNativeUnit: NativeUnit,
    afterSwitchHeightSample: Length,
    afterSwitchNativeUnitSample: NativeUnit
) : NativeUnitModel<Length>(Length.kZero) {

    private val switchHeight = switchHeight.value
    private val switchNativeUnit = switchNativeUnit.value
    private val beforeSwitchSlope = switchNativeUnit.value / switchHeight.value
    private val afterSwitchSlope = (afterSwitchNativeUnitSample.value - switchNativeUnit.value) /
        (afterSwitchHeightSample.value - switchHeight.value)

    override fun fromNativeUnitPosition(nativeUnits: Double): Double =
        when {
            nativeUnits < switchNativeUnit -> nativeUnits / beforeSwitchSlope
            else -> {
                val afterSwitchNativeUnits = nativeUnits - switchNativeUnit
                val afterSwitchHeight = afterSwitchNativeUnits / afterSwitchSlope
                switchHeight + afterSwitchHeight
            }
        }

    override fun toNativeUnitPosition(modelledUnit: Double): Double =
        when {
            modelledUnit < switchHeight -> modelledUnit * beforeSwitchSlope
            else -> {
                val afterSwitchHeight = modelledUnit - switchHeight
                val afterSwitchNativeUnits = afterSwitchHeight * afterSwitchSlope
                switchNativeUnit + afterSwitchNativeUnits
            }
        }

    override fun toNativeUnitError(modelledUnit: Double): Double =
        modelledUnit * afterSwitchSlope

    override fun fromNativeUnitVelocity(nativeUnitVelocity: Double): Double =
        nativeUnitVelocity / afterSwitchSlope

    override fun toNativeUnitVelocity(modelledUnitVelocity: Double): Double =
        modelledUnitVelocity * afterSwitchSlope

}
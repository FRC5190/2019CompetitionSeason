package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.STU

object CascadingElevatorHelper {

    fun getHeightFromNativeUnit(nativeUnits: NativeUnit) = getHeightFromNativeUnit(nativeUnits.value).meter
    private fun getHeightFromNativeUnit(nativeUnits: Double) = when {
        nativeUnits < Constants.kNativeUnitOfCascadingSwitch.value ->
            Constants.kHeightOfCascadingSwitch.value / Constants.kNativeUnitOfCascadingSwitch.value * nativeUnits
        else -> {
            val afterSwitchNativeUnits = nativeUnits - Constants.kNativeUnitOfCascadingSwitch.value
            val afterSwitchHeight =
                (Constants.kHeightOfCascadingSwitch.value / Constants.kNativeUnitOfCascadingSwitch.value) * 2 * afterSwitchNativeUnits
            Constants.kHeightOfCascadingSwitch.value + afterSwitchHeight
        }
    }

    fun getNativeUnitsFromHeight(height: Length) = getNativeUnitsFromHeight(height.value).STU
    private fun getNativeUnitsFromHeight(height: Double): Double = when {
        height < Constants.kHeightOfCascadingSwitch.value ->
            Constants.kNativeUnitOfCascadingSwitch.value / Constants.kHeightOfCascadingSwitch.value * height
        else -> {
            val afterSwitchHeight = height - Constants.kHeightOfCascadingSwitch.value
            val afterSwitchNativeUnits =
                (Constants.kNativeUnitOfCascadingSwitch.value / Constants.kHeightOfCascadingSwitch.value) / 2 * afterSwitchHeight
            Constants.kNativeUnitOfCascadingSwitch.value + afterSwitchNativeUnits
        }
    }

}
package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.STUPer100ms
import org.ghrobotics.lib.mathematics.units.nativeunits.fromNativeUnitVelocity
import org.ghrobotics.lib.wrappers.ctre.FalconSRX
import kotlin.properties.Delegates

class SpringCascadeFalconSRX(
    id: Int,
    private val model: SpringCascadeNativeUnitModel,
    timeout: Time = 10.millisecond
) : FalconSRX<Length>(id, model, timeout) {
    override var allowedClosedLoopError by Delegates.observable(Length(0.0)) { _, _, newValue ->
        // Using native unit velocity from SpringCascadeNativeUnitModel will give us correct error since it wont
        // have the offset
        configAllowableClosedloopError(0, model.fromNativeUnitVelocity(newValue.value).toInt(), timeoutInt)
    }
    val activeTrajectoryVelocity: LinearVelocity
        get() = super.getActiveTrajectoryVelocity().STUPer100ms.fromNativeUnitVelocity(model)
}
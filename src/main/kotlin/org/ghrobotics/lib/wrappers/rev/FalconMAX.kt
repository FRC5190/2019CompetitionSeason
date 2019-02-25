package org.ghrobotics.lib.wrappers.rev

import com.revrobotics.CANEncoder
import com.revrobotics.CANPIDController
import com.revrobotics.CANSparkMax
import com.revrobotics.ControlType
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.Volt
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.wrappers.FalconMotor
import kotlin.properties.Delegates

class FalconMAX<T : SIUnit<T>>(
    id: Int,
    type: MotorType = MotorType.kBrushless,
    private val model: NativeUnitModel<T>
) :
    CANSparkMax(id, type),
    FalconMotor<T> {

    private val controller = CANPIDController(this)
    private val enc = CANEncoder(this)

    override var percentOutput: Double
        get() = this.appliedOutput
        set(value) {
            set(value)
        }

    var kP by Delegates.observable(0.0) { _, _, newValue -> controller.p = newValue }
    var kI by Delegates.observable(0.0) { _, _, newValue -> controller.i = newValue }
    var kD by Delegates.observable(0.0) { _, _, newValue -> controller.d = newValue }
    var kF by Delegates.observable(0.0) { _, _, newValue -> controller.ff = newValue }

    var brakeMode by Delegates.observable(IdleMode.kBrake) { _, _, newValue ->
        idleMode = newValue
    }

    var voltageCompensationSaturation by Delegates.observable(12.volt) { _, _, newValue ->
        enableVoltageCompensation(newValue.value)
    }

    var currentLimit by Delegates.observable(0.amp) { _, _, newValue -> setSmartCurrentLimit(newValue.value.toInt()) }

    override var velocity: Velocity<T>
        get() = model.fromNativeUnitVelocity((enc.velocity / 60).nativeUnits.velocity)
        set(value) {
            controller.setReference(
                model.toNativeUnitVelocity(value).value * 60,
                ControlType.kVelocity
            )
        }

    override val voltageOutput: Volt
        get() = appliedOutput.volt

    override fun setVelocityAndArbitraryFeedForward(velocity: Velocity<T>, arbitraryFeedForward: Double) {
        controller.setReference(
            model.toNativeUnitVelocity(velocity).value * 60,
            ControlType.kVelocity,
            0,
            arbitraryFeedForward * 12
        )
    }
}
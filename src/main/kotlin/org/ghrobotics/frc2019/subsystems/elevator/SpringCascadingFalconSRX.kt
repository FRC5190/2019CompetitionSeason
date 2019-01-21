package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.ghrobotics.lib.mathematics.units.nativeunits.fromModel
import org.ghrobotics.lib.wrappers.ctre.AbstractFalconSRX
import kotlin.properties.Delegates

class SpringCascadingFalconSRX(
    id: Int,
    private val model: SpringCascadingNativeUnitModel,
    timeout: Time = 10.millisecond
) : AbstractFalconSRX<Length>(id, timeout) {
    override var allowedClosedLoopError by Delegates.observable(model.zero) { _, _, newValue ->
        configAllowableClosedloopError(0, model.fromOverallModel(newValue.value).toInt(), timeoutInt)
    }
    override var motionCruiseVelocity by Delegates.observable(model.zero.velocity) { _, _, newValue ->
        configMotionCruiseVelocity((model.fromOverallModel(newValue.value) / 10.0).toInt(), timeoutInt)
    }
    override var motionAcceleration by Delegates.observable(model.zero.acceleration) { _, _, newValue ->
        configMotionAcceleration((model.fromOverallModel(newValue.value) / 10.0).toInt(), timeoutInt)
    }
    override var sensorPosition: Length
        get() = getSelectedSensorPosition(0).STU.toModel(model)
        set(value) {
            setSelectedSensorPosition(value.fromModel(model).value.toInt(), 0, timeoutInt)
        }
    override val sensorVelocity get() = model.toOverallModel(getSelectedSensorVelocity(0) * 10.0).meter.velocity

    val activeTrajectoryVelocity: LinearVelocity
        get() = model.toOverallModel(super.getActiveTrajectoryVelocity() * 10.0).meter.velocity

    override fun set(controlMode: ControlMode, length: Length) = set(controlMode, length.fromModel(model).value)

    override fun set(controlMode: ControlMode, velocity: LinearVelocity) =
        set(controlMode, velocity, DemandType.ArbitraryFeedForward, 0.0)

    override fun set(controlMode: ControlMode, length: Length, demandType: DemandType, outputPercent: Double) =
        set(controlMode, length.fromModel(model).value, demandType, outputPercent)

    override fun set(
        controlMode: ControlMode,
        velocity: LinearVelocity,
        demandType: DemandType,
        outputPercent: Double
    ) = set(controlMode, model.fromOverallModel(velocity.value) / 10.0, demandType, outputPercent)
}
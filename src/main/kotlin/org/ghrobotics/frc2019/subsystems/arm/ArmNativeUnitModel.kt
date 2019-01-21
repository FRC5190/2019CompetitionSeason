package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.radian

class ArmNativeUnitModel(
    armOffsetNativeUnits: NativeUnit,
    armOffsetAngle: Rotation2d,
    sensorUnitsPerRotation: NativeUnit
) : NativeUnitModel<Rotation2d>(sensorUnitsPerRotation) {

    private val armOffsetNativeUnits = armOffsetNativeUnits.value
    private val armOffsetAngle = armOffsetAngle.radian

    override fun createNew(newValue: Double) = newValue.radian

    override fun toModel(value: Double): Double {
        val valueFromOffset = value - armOffsetNativeUnits
        val angleFromOffset = 2.0 * Math.PI * (valueFromOffset / _sensorUnitsPerRotation)
        return angleFromOffset + armOffsetAngle
    }

    override fun fromModel(value: Double): Double {
        val valueFromOffset = value - armOffsetAngle
        val nativeUnitsFromOffset = (valueFromOffset / (2.0 * Math.PI)) * _sensorUnitsPerRotation
        return nativeUnitsFromOffset + armOffsetNativeUnits
    }

}
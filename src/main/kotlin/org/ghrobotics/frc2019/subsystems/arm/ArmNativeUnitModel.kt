package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel

class ArmNativeUnitModel(
    armOffsetNativeUnits: NativeUnit,
    armOffsetAngle: Rotation2d,
    sensorUnitsPerRotation: NativeUnit
) : NativeUnitModel<Rotation2d>(Rotation2d(0.0)) {

    private val armOffsetNativeUnits = armOffsetNativeUnits.value
    private val armOffsetAngle = armOffsetAngle.radian
    private val nativeUnitsPerRotation = sensorUnitsPerRotation.value

    override fun fromNativeUnitPosition(nativeUnits: Double): Double {
        val valueFromOffset = nativeUnits - armOffsetNativeUnits
        val angleFromOffset = 2.0 * Math.PI * (valueFromOffset / nativeUnitsPerRotation)
        return angleFromOffset + armOffsetAngle
    }

    override fun toNativeUnitPosition(modelledUnit: Double): Double {
        val valueFromOffset = modelledUnit - armOffsetAngle
        val nativeUnitsFromOffset = (valueFromOffset / (2.0 * Math.PI)) * nativeUnitsPerRotation
        return nativeUnitsFromOffset + armOffsetNativeUnits
    }

    override fun toNativeUnitError(modelledUnit: Double): Double =
        (modelledUnit / (Math.PI * 2.0)) * nativeUnitsPerRotation

    override fun fromNativeUnitVelocity(nativeUnitVelocity: Double): Double =
        (nativeUnitVelocity / nativeUnitsPerRotation) * (Math.PI * 2.0)

    override fun toNativeUnitVelocity(modelledUnitVelocity: Double): Double =
        (modelledUnitVelocity / (Math.PI * 2.0)) * nativeUnitsPerRotation

}
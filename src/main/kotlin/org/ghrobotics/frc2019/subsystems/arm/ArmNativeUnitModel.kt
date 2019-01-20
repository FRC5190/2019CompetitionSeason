package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel
//
//class ArmNativeUnitModel(
//    private val armOffsetTicks: NativeUnit,
//    private val armOffsetAngle: Rotation2d,
//    sensorUnitsPerRotation: NativeUnit
//) : NativeUnitModel<Rotation2d>(sensorUnitsPerRotation) {
//    override fun createNew(newValue: Double) = Rotation2d(newValue)
//
//    override fun toModel(value: Double) = 2.0 * Math.PI * value / _sensorUnitsPerRotation
//    override fun fromModel(value: Double) = _sensorUnitsPerRotation * value / (2.0 * Math.PI)
//}
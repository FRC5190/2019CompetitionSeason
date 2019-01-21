package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class ArmNativeUnitModelTest {

    private val model1 = ArmNativeUnitModel(
        armOffsetNativeUnits = 512.STU,
        armOffsetAngle = 90.degree,
        sensorUnitsPerRotation = 1024.STU
    )

    private val model2 = ArmNativeUnitModel(
        armOffsetNativeUnits = 500.STU,
        armOffsetAngle = 90.degree,
        sensorUnitsPerRotation = 1024.STU
    )

    @Test
    fun testFromModel1() {
        val one = 0.degree
        val two = 90.degree
        val three = 180.degree

        val oneNativeUnit = model1.fromModel(one)
        val twoNativeUnit = model1.fromModel(two)
        val threeNativeUnit = model1.fromModel(three)

        assert(oneNativeUnit.value epsilonEquals 256.0)
        assert(twoNativeUnit.value epsilonEquals 512.0)
        assert(threeNativeUnit.value epsilonEquals 768.0)
    }

    @Test
    fun testFromModel2() {
        val one = 0.degree
        val two = 90.degree
        val three = 180.degree

        val oneNativeUnit = model2.fromModel(one)
        val twoNativeUnit = model2.fromModel(two)
        val threeNativeUnit = model2.fromModel(three)

        assert(oneNativeUnit.value epsilonEquals 244.0)
        assert(twoNativeUnit.value epsilonEquals 500.0)
        assert(threeNativeUnit.value epsilonEquals 756.0)
    }

    @Test
    fun testToModel1() {
        val one = 256.STU
        val two = 512.STU
        val three = 768.STU

        val oneAngle = model1.toModel(one)
        val twoAngle = model1.toModel(two)
        val threeAngle = model1.toModel(three)

        assert(oneAngle.degree epsilonEquals 0.0)
        assert(twoAngle.degree epsilonEquals 90.0)
        assert(threeAngle.degree epsilonEquals 180.0)
    }

    @Test
    fun testToModel2() {
        val one = 244.STU
        val two = 500.STU
        val three = 756.STU

        val oneAngle = model2.toModel(one)
        val twoAngle = model2.toModel(two)
        val threeAngle = model2.toModel(three)

        assert(oneAngle.degree epsilonEquals 0.0)
        assert(twoAngle.degree epsilonEquals 90.0)
        assert(threeAngle.degree epsilonEquals 180.0)
    }

}
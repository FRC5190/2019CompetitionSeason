package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import org.junit.Test

class ArmNativeUnitModelTest {

    private val model1 = ArmNativeUnitModel(
        armOffsetNativeUnits = 512.STU,
        armOffsetAngle = 90.degree,
        sensorResolution = 1024.STU
    )

    private val model2 = ArmNativeUnitModel(
        armOffsetNativeUnits = (-512).STU,
        armOffsetAngle = 90.degree,
        sensorResolution = 1024.STU
    )

    @Test
    fun testToNativeUnit1() {
        val one = (-225).degree
        val two = (-180).degree
        val three = 0.degree
        val four = 180.degree
        val five = 225.degree

        val oneNativeUnit = model1.toNativeUnitPosition(one)
        val twoNativeUnit = model1.toNativeUnitPosition(two)
        val threeNativeUnit = model1.toNativeUnitPosition(three)
        val fourNativeUnit = model1.toNativeUnitPosition(four)
        val fiveNativeUnit = model1.toNativeUnitPosition(five)

        assert(oneNativeUnit.value epsilonEquals 640.0)
        assert(twoNativeUnit.value epsilonEquals 768.0)
        assert(threeNativeUnit.value epsilonEquals 256.0)
        assert(fourNativeUnit.value epsilonEquals 768.0)
        assert(fiveNativeUnit.value epsilonEquals 896.0)
    }

    @Test
    fun testToNativeUnitVelocity1() {
        val one = 0.degree.velocity
        val two = 90.degree.velocity
        val three = 180.degree.velocity

        val oneNativeUnit = model1.toNativeUnitVelocity(one)
        val twoNativeUnit = model1.toNativeUnitVelocity(two)
        val threeNativeUnit = model1.toNativeUnitVelocity(three)

        assert(oneNativeUnit.value epsilonEquals 0.0)
        assert(twoNativeUnit.value epsilonEquals 256.0)
        assert(threeNativeUnit.value epsilonEquals 512.0)
    }

    @Test
    fun testToNativeUnit2() {
        val one = (-225).degree
        val two = (-180).degree
        val three = 0.degree
        val four = 180.degree
        val five = 225.degree

        val oneNativeUnit = model2.toNativeUnitPosition(one)
        val twoNativeUnit = model2.toNativeUnitPosition(two)
        val threeNativeUnit = model2.toNativeUnitPosition(three)
        val fourNativeUnit = model2.toNativeUnitPosition(four)
        val fiveNativeUnit = model2.toNativeUnitPosition(five)

        assert(oneNativeUnit.value epsilonEquals -640.0)
        assert(twoNativeUnit.value epsilonEquals -768.0)
        assert(threeNativeUnit.value epsilonEquals -256.0)
        assert(fourNativeUnit.value epsilonEquals -768.0)
        assert(fiveNativeUnit.value epsilonEquals -896.0)
    }

    @Test
    fun testFromNativeUnit1() {
        val one = 256.STU
        val two = 512.STU
        val three = 768.STU

        val oneAngle = model1.fromNativeUnitPosition(one)
        val twoAngle = model1.fromNativeUnitPosition(two)
        val threeAngle = model1.fromNativeUnitPosition(three)

        assert(oneAngle.degree epsilonEquals 0.0)
        assert(twoAngle.degree epsilonEquals 90.0)
        assert(threeAngle.degree epsilonEquals 180.0)
    }

    @Test
    fun testFromNativeUnitVelocity1() {
        val one = 256.STU.velocity
        val two = 512.STU.velocity
        val three = 768.STU.velocity

        val oneAngle = model1.fromNativeUnitVelocity(one)
        val twoAngle = model1.fromNativeUnitVelocity(two)
        val threeAngle = model1.fromNativeUnitVelocity(three)

        assert(oneAngle.value epsilonEquals Math.PI * 0.5)
        assert(twoAngle.value epsilonEquals Math.PI)
        assert(threeAngle.value epsilonEquals Math.PI * 1.5)
    }

    @Test
    fun testFromNativeUnit2() {
        val one = ( -640).STU
        val two = (-768).STU
        val three = (-256).STU
        val four = (-896.0).STU

        val oneAngle = model2.fromNativeUnitPosition(one)
        val twoAngle = model2.fromNativeUnitPosition(two)
        val threeAngle = model2.fromNativeUnitPosition(three)
        val fourAngle = model2.fromNativeUnitPosition(four)

        assert(oneAngle epsilonEquals (-225).degree)
        assert(twoAngle epsilonEquals (-180).degree)
        assert(threeAngle epsilonEquals 0.degree)
        assert(fourAngle epsilonEquals 225.degree)
    }

}
@file:Suppress("MatchingDeclarationName")

package org.ghrobotics.frc2019.vision

import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.second
import kotlin.concurrent.fixedRateTimer

object JeVoisManager {

    private val kJevoisGson = Gson()

    private val connectedJeVoisCameras = mutableListOf<JeVois>()

    var isFrontJeVoisConnected = false
        private set

    var isBackJeVoisConnected = false
        private set

    init {
        fixedRateTimer(name = "JevoisManager", period = 10000L) {
            val currentTime = Timer.getFPGATimestamp().second

            connectedJeVoisCameras.removeIf {
                it.update(currentTime)
                if (!it.isAlive) {
                    println("[JeVois Manager] Disconnected Camera: ${it.systemPortName}")
                    it.dispose()
                    true
                } else {
                    false
                }
            }

            val jeVoisSerialPorts = SerialPort.getCommPorts()
                .filter { it.descriptivePortName.contains("JeVois", true) }

            for (serialPort in jeVoisSerialPorts) {
                if (connectedJeVoisCameras.any { it.systemPortName.equals(serialPort.systemPortName, true) }) {
                    continue
                }
                println("[JeVois Manager] Found new camera: ${serialPort.systemPortName}")
                connectedJeVoisCameras.add(JeVois(serialPort))
            }

            isFrontJeVoisConnected = connectedJeVoisCameras.any { it.isFront == true }
            isBackJeVoisConnected = connectedJeVoisCameras.any { it.isFront == false }
        }
    }

    class JeVois(
        private val serialPort: SerialPort
    ) {

        var isFront: Boolean? = null
            private set

        val systemPortName: String = serialPort.systemPortName

        private var lastMessageReceived = 0.second
        private var wasUnplugged = false

        var isAlive = true
            private set

        init {
            serialPort.openPort()
            serialPort.addDataListener(object : SerialPortDataListener {
                private var stringBuffer = StringBuilder(1024)

                override fun serialEvent(event: SerialPortEvent) {
                    try {
                        if (event.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
                            return
                        val bytesAvailable = serialPort.bytesAvailable()
                        if (bytesAvailable < 0) {
                            wasUnplugged = true
                            return
                        }
                        val newData = ByteArray(bytesAvailable)
                        serialPort.readBytes(newData, newData.size.toLong())
                        for (newByte in newData) {
                            val newChar = newByte.toChar()
                            if (newChar != '\n') {
                                stringBuffer.append(newChar)
                                continue
                            }
                            processMessage(stringBuffer.toString())
                            stringBuffer.clear()
                        }
                    } catch (e: Throwable) {
                        println("[JeVois] ${e.localizedMessage}")
//                    e.printStackTrace()
                    }
                }

                override fun getListeningEvents() = SerialPort.LISTENING_EVENT_DATA_AVAILABLE
            })
            lastMessageReceived = Timer.getFPGATimestamp().second
        }

        fun processMessage(message: String) {
            lastMessageReceived = Timer.getFPGATimestamp().second
            if (!message.startsWith('{')) return
            try {
                val jsonData = kJevoisGson.fromJson<JsonObject>(message)

                val isFront = jsonData["is_front"].asBoolean
                val timestamp = Timer.getFPGATimestamp() - jsonData["capture_ago"].asDouble
                val contours = jsonData["targets"].asJsonArray
                    .filterIsInstance<JsonObject>()

                this.isFront = isFront

                VisionProcessing.processData(VisionData(isFront, timestamp, contours))
            } catch (e: JsonParseException) {
//            e.printStackTrace()
                println("[JeVois] Got Invalid Data: $message")
            }
        }

        fun update(currentTime: Time) {
            isAlive = !wasUnplugged && currentTime - lastMessageReceived <= Constants.kVisionCameraTimeout
        }

        fun dispose() {
            serialPort.closePort()
        }

        private fun SerialPort.writeString(data: String) = writeBytes(data.toByteArray())
        private fun SerialPort.writeBytes(data: ByteArray) = writeBytes(data, data.size.toLong())

    }

}

data class VisionData(
    val isFront: Boolean,
    val timestamp: Double,
    val targets: List<JsonObject>
)
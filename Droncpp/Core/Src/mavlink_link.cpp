/*
 * mavlink_link.cpp
 *
 *  Created on: 19 jul 2026
 *      Author: specter0163
 */

#include "mavlink_link.hpp"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"


MavlinkLink::MavlinkLink(){}

void MavlinkLink::SendMessage(mavlink_message_t &msg){
	uint16_t len = mavlink_msg_to_send_buffer(sendBuffer_, &msg);
	CDC_Transmit_FS(sendBuffer_, len);
}

void MavlinkLink::SendHeartbeat(){
	mavlink_message_t msg;

	mavlink_msg_heartbeat_pack(kSystemId, kComponentId, &msg,
			MAV_TYPE_QUADROTOR,		// tipo de vehículo — ajusta si no es quadcopter
			MAV_AUTOPILOT_GENERIC,  // autopiloto genérico (no es un ArduPilot/PX4 real)
			0,						// base_mode
			0,						// custom_mode
			MAV_STATE_ACTIVE);		// estado del sistema

	SendMessage(msg);
}

void MavlinkLink::SendAttitude(float roll, float pitch, float yaw,
							   float rollspeed, float pitchspeed, float yawspeed){
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(kSystemId, kComponentId, &msg,
							  HAL_GetTick(),
							  roll, pitch, yaw,
							  rollspeed, pitchspeed, yawspeed);
	SendMessage(msg);

}

void MavlinkLink::SendGlobalPosition(double lat, double lon, float alt, float relativeAlt,
									 float vx, float vy, float vz, float heading_deg){
	mavlink_message_t msg;

    mavlink_msg_global_position_int_pack(kSystemId, kComponentId, &msg,
        HAL_GetTick(),                    // time_boot_ms
        static_cast<int32_t>(lat * 1e7),  // lat, 1E7 grados
        static_cast<int32_t>(lon * 1e7),  // lon, 1E7 grados
        static_cast<int32_t>(alt * 1000), // alt, mm sobre el nivel del mar
        static_cast<int32_t>(relativeAlt * 1000), // alt relativa al home, mm
        static_cast<int16_t>(vx * 100),   // vx, cm/s
        static_cast<int16_t>(vy * 100),   // vy, cm/s
        static_cast<int16_t>(vz * 100),   // vz, cm/s
        static_cast<uint16_t>(heading_deg * 100)); // heading, centi-grados (0-35999)

    SendMessage(msg);
}

void MavlinkLink::SendSysStatus(uint16_t voltage_mV, int16_t current_10mA, int8_t remaining_pct){
    mavlink_message_t msg;

    mavlink_msg_sys_status_pack(kSystemId, kComponentId, &msg,
        0, 0, 0,           // sensores presentes/habilitados/con salud (0 = no reportamos detalle todavía)
        0,                 // load del sistema (0-1000, no lo medimos por ahora)
        voltage_mV,
        current_10mA,
        remaining_pct,
        0, 0, 0, 0, 0, 0,  // contadores de drop/errores de comunicación (0 por ahora)
        0, 0, 0);          // sensores presentes/habilitados/con salud extendidos (MAVLink 2)

    SendMessage(msg);
}

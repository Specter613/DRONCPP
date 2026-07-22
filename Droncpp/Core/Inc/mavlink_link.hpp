/*
 * mavlink_link.hpp
 *
 *  Created on: 19 jul 2026
 *      Author: specter0163
 */

#ifndef INC_MAVLINK_LINK_HPP_
#define INC_MAVLINK_LINK_HPP_

#include "cstdint"

extern "C"{
#include "common/mavlink.h"
}

class MavlinkLink{
public:
	MavlinkLink();

	void SendHeartbeat();

	 // roll,pitch,yaw en radianes; rollspeed,pitchspeed,yawspeed en rad/s (velocidad angular)
	void SendAttitude(float roll, float pitch, float yaw,
					  float rollspeed, float pitchspeed, float yawspeed);

    // lat,lon en grados (se convierten internamente a 1e7); alt en metros;
    // vx,vy,vz en m/s (velocidad NED)
	void SendGlobalPosition(double lat, double lon, float alt, float relativeAlt,
							float vx, float vy, float vz, float heading_deg);
	// voltage en mV, current en 10*mA (o -1 si no disponible), remaining en % (0-100, o -1 si no disponible)
	void SendSysStatus(uint16_t voltage_mV, int16_t current_10mA, int8_t remaining_pct);

	void SendOpticalFlow(float distance_mm, float flowX, float flowY, uint8_t quality);
private:
	void SendMessage(mavlink_message_t &msg);

	static constexpr uint8_t kSystemId = 1;
	static constexpr uint8_t kComponentId = MAV_COMP_ID_AUTOPILOT1;

	uint8_t sendBuffer_[MAVLINK_MAX_PACKET_LEN];
};


#endif /* INC_MAVLINK_LINK_HPP_ */

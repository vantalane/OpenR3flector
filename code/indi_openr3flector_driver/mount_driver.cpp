/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.
 Copyright (C) 2026 vantalane <mete@kestech.net>
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
******************************************************************************
 *
 */

#include "mount_driver.h"

#include <libnova/julian_day.h>
#include <libnova/ln_types.h>
#include <libnova/sidereal_time.h>
#include <libnova/transform.h>
#include <libnova/utility.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>

// #include "SGP4.h"

#include "connectionplugins/connectionserial.h"
#include "defaultdevice.h"
#include "indiapi.h"
#include "indibasetypes.h"
#include "indicom.h"
#include "indidevapi.h"
#include "indiguiderinterface.h"
#include "indilogger.h"
#include "inditelescope.h"
#include "libastro.h"

extern "C"
{
#include "SGP4.h"
}

// Single unique pointer to the driver.
static std::unique_ptr<MountDriver> telescope_sim(new MountDriver());

MountDriver::MountDriver() : GI(this)
{
	// Let's specify the driver version
	setVersion(1, 0);

	SetTelescopeCapability(
		TELESCOPE_CAN_PARK | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT |
			TELESCOPE_CAN_SYNC | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_TIME |
			TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK |
			TELESCOPE_CAN_TRACK_SATELLITE,
		1);

	setTelescopeConnection(CONNECTION_SERIAL);
}

const char* MountDriver::getDefaultName()
{
	return "OpenR3flector device";
}

bool MountDriver::initProperties()
{
	// Make sure to init parent properties first
	INDI::Telescope::initProperties();
	INDI::GuiderInterface::initProperties("r3stuff");

	IUFillNumber(&TargetNP[0], "CURRENT_AZ", "Az (°)", "%.2f", 0, 360, 0, 0);
	IUFillNumber(&TargetNP[1], "CURRENT_ALT", "Alt (°)", "%.2f", 0, 90, 0, 0);

	TargetNP.fill(getDeviceName(), "HORIZONTAL_COORD", "Az/Alt",
				  MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

	DeviceStateTP[0].fill("DEVICE_STATE", "State", "Unknown");
	DeviceStateTP.fill(getDeviceName(), "DEVICE_STATUS", "Device Status",
					   MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

	/*Home buttons*/
	HomingRunSP[0].fill("HOME_SAFE_RUN", "Safe", ISS_OFF);
	HomingRunSP[1].fill("HOME_UNSAFE_RUN", "Unsafe", ISS_OFF);
	HomingRunSP.fill(getDeviceName(), "HOMING_BUTTONS", "Run homing proc",
					 MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 2, IPS_IDLE);

	/*HW reset button*/
	ResetButtonSP[0].fill("HW_RESET", "HW Reset", ISS_OFF);
	ResetButtonSP.fill(getDeviceName(), "HW_RESET_BTN", "Reset hardware",
					   MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 2, IPS_IDLE);

	/*Pointing model reset button*/
	ResetSyncSP[0].fill("RESET_SYNC", "Reset Sync Points", ISS_OFF);
	ResetSyncSP.fill(getDeviceName(), "SYNC_RESET_BTN", "Pointing Model",
					 MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

	/*Sync point count display*/
	SyncPointCountNP[0].fill("SYNC_POINT_COUNT", "Sync Points Loaded", "%.0f",
							 0, 3, 1, 0);
	SyncPointCountNP.fill(getDeviceName(), "SYNC_POINT_COUNT_PROP",
						  "Pointing Model", MAIN_CONTROL_TAB, IP_RO, 60,
						  IPS_IDLE);

	// Read-only current motion state
	MotionStateNP[0].fill("AZ_VELOCITY", "Az Velocity (°/s)", "%.3f", 0, 50, 0,
						  0);
	MotionStateNP[1].fill("ALT_VELOCITY", "Alt Velocity (°/s)", "%.3f", 0, 50,
						  0, 0);
	MotionStateNP[2].fill("AZ_ACCEL", "Az Accel (°/s²)", "%.3f", 0, 50, 0, 0);
	MotionStateNP[3].fill("ALT_ACCEL", "Alt Accel (°/s²)", "%.3f", 0, 50, 0, 0);
	MotionStateNP.fill(getDeviceName(), "MOTION_STATE", "Motion State",
					   MOTION_TAB, IP_RO, 60, IPS_IDLE);

	// Read-write limits
	MotionLimitsNP[0].fill("MAX_AZ_VELOCITY", "Max Az Velocity (°/s)", "%.2f",
						   0, 25, 0.5, 15);
	MotionLimitsNP[1].fill("MAX_ALT_VELOCITY", "Max Alt Velocity (°/s)", "%.2f",
						   0, 25, 0.5, 15);
	MotionLimitsNP[2].fill("MAX_AZ_ACCEL", "Max Az Accel (°/s²)", "%.2f", 0.05,
						   35, 0.5, 15);
	MotionLimitsNP[3].fill("MAX_ALT_ACCEL", "Max Alt Accel (°/s²)", "%.2f",
						   0.05, 35, 0.5, 15);
	MotionLimitsNP.fill(getDeviceName(), "MOTION_LIMITS", "Motion Limits",
						MOTION_TAB, IP_RW, 60, IPS_IDLE);

	TrackSatSP[0].fill("SAT_TRACK_ON", "Enable", ISS_OFF);
	TrackSatSP[1].fill("SAT_TRACK_OFF", "Disable", ISS_ON);
	TrackSatSP.fill(getDeviceName(), "SATELLITE_TRACK", "Satellite Tracking",
					"Satellite", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

	SatTrackLeadSecondsNP[0].fill("SAT_TRACK_TIME_OFFSET", "Time Offset",
								  "%0.5f", -10, 10, 0.1, 0);
	SatTrackLeadSecondsNP.fill(getDeviceName(), "SAT_TRACK_TIME",
							   "Seconds offset", "Satellite", IP_RW, 60,
							   IPS_IDLE);

	/*LNB power / polarisation*/
	LNBPowerSP[0].fill("LNB_OFF", "Off", ISS_ON);
	LNBPowerSP[1].fill("LNB_VERTICAL", "Vertical", ISS_OFF);
	LNBPowerSP[2].fill("LNB_HORIZONTAL", "Horizontal", ISS_OFF);
	LNBPowerSP.fill(getDeviceName(), "LNB_POWER", "LNB Power", "Receiver",
					IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

	/*LNB local oscillator band*/
	LNBLocalOscSP[0].fill("LNB_LO_LOW", "Low (9.75 GHz)", ISS_ON);
	LNBLocalOscSP[1].fill("LNB_LO_HIGH", "High (10.60 GHz)", ISS_OFF);
	LNBLocalOscSP.fill(getDeviceName(), "LNB_LO", "LNB Local Osc", "Receiver",
					   IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

	/*Pointing-model calibration offsets (read-only, from firmware status)*/
	CalibOffsetsNP[0].fill("AZ_BODY_OFFSET", "Az Body Offset (°)", "%.4f", -180,
						   180, 0, 0);
	CalibOffsetsNP[1].fill("ALT_BODY_OFFSET", "Alt Body Offset (°)", "%.4f",
						   -90, 90, 0, 0);
	CalibOffsetsNP[2].fill("AZ_0_TILT", "Az 0° Tilt (°)", "%.4f", -90, 90, 0,
						   0);
	CalibOffsetsNP[3].fill("AZ_90_TILT", "Az 90° Tilt (°)", "%.4f", -90, 90, 0,
						   0);
	CalibOffsetsNP[4].fill("CALIB_BITMAP", "Calib Points Bitmap", "%.0f", 0,
						   255, 1, 0);
	CalibOffsetsNP.fill(getDeviceName(), "CALIB_OFFSETS", "Pointing Offsets",
						MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

	// The mount is initially in IDLE state.
	TrackState = SCOPE_PARKED;

	MountTypeSP = MOUNT_ALTAZ;
	SetParkDataType(PARK_SIMPLE);

	// Add debug controls
	addDebugControl();

	// Set the driver interface to indicate that we can also do pulse guiding
	setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

	// We want to query the mount every 500ms by default. The user can override
	// this value.
	setDefaultPollingPeriod(500);

	return true;
}

bool MountDriver::updateProperties()
{
	INDI::Telescope::updateProperties();

	if (isConnected())
	{
		defineProperty(TargetNP);
		defineProperty(DeviceStateTP);
		defineProperty(MotionStateNP);
		defineProperty(MotionLimitsNP);
		// defineProperty(SatelliteTLETP);
		defineProperty(TrackSatSP);
		defineProperty(HomingRunSP);
		defineProperty(ResetButtonSP);
		defineProperty(ResetSyncSP);
		defineProperty(SyncPointCountNP);
		defineProperty(SatTrackLeadSecondsNP);
		defineProperty(LNBPowerSP);
		defineProperty(LNBLocalOscSP);
		defineProperty(CalibOffsetsNP);
	}
	else
	{
		deleteProperty(TargetNP);
		deleteProperty(DeviceStateTP);
		deleteProperty(MotionStateNP);
		deleteProperty(MotionLimitsNP);
		// deleteProperty(SatelliteTLETP);
		deleteProperty(TrackSatSP);
		deleteProperty(HomingRunSP);
		deleteProperty(ResetButtonSP);
		deleteProperty(ResetSyncSP);
		deleteProperty(SyncPointCountNP);
		deleteProperty(SatTrackLeadSecondsNP);
		deleteProperty(LNBPowerSP);
		deleteProperty(LNBLocalOscSP);
		deleteProperty(CalibOffsetsNP);
	}

	return true;
}

bool MountDriver::Handshake()
{
	// This function is ensure that we have communication with the mount
	// Below we send it 0x6 byte and check for 'S' in the return. Change this
	// to be valid for your driver. It could be anything, you can simply put
	// this below return readScopeStatus() since this will try to read the
	// position and if successful, then communicatoin is OK.

	if (ReadScopeStatus() == false)
	{
		LOG_ERROR("Failed to read scope status during handshake");
		return false;
	}

	// char cmd[DRIVER_LEN] = {0};
	// snprintf(cmd, sizeof(cmd), "%c\n", CMD_HOME);
	// if (sendCommand(cmd) == false)
	// {
	// 	return false;
	// }

	char response[DRIVER_LEN] = {0};
	if (device_state == DEV_STATE_UNINIT)
	{
		if (!sendCommand("h\n", response, -1, -1))
		{
			LOG_ERROR("Failed to send home command during handshake");
			return false;
		}

		// Wait a bit for homing to start
		usleep(100000);	 // 100ms

		// Read status again
		if (!ReadScopeStatus()) return false;
	}

	LOG_INFO("Ran handshake");
	return true;
}

bool MountDriver::ReadScopeStatus()
{
	// std::lock_guard<std::mutex> lock(m_Mutex);

	char cmd[DRIVER_LEN] = {0};
	char res[DRIVER_LEN] = {0};

	snprintf(cmd, sizeof(cmd), "%c\n", CMD_GET_STATUS);

	if (sendCommand(cmd, res, -1, -1) == false)
	{
		return false;
	}

	float az_body_offset, alt_body_offset, az_0_tilt, az_90_tilt;
	uint32_t calib_bitmap;

	/*System state, AZ max XL, ALT max XL, AZ max VELO, ALT max VELO */
	uint32_t raw_state = 0;
	sscanf(res, "%u,%f,%f,%f,%f,%f,%f,%f,%f,%u", &raw_state, &m_max_az_acc,
		   &m_max_alt_acc, &m_max_az_velocity, &m_max_alt_velocity,
		   &az_body_offset, &alt_body_offset, &az_0_tilt, &az_90_tilt,
		   &calib_bitmap);

	device_state = static_cast<DeviceState_t>(raw_state);

	CalibOffsetsNP[0].setValue(az_body_offset);
	CalibOffsetsNP[1].setValue(alt_body_offset);
	CalibOffsetsNP[2].setValue(az_0_tilt);
	CalibOffsetsNP[3].setValue(az_90_tilt);
	CalibOffsetsNP[4].setValue(calib_bitmap);
	CalibOffsetsNP.setState(IPS_OK);
	CalibOffsetsNP.apply();

	/*get motor stuff*/
	/*az pos, az setpoint,alt pos, alt setpoint, az velo, alt velo, az accel,
	 * velo accel, */
	snprintf(cmd, sizeof(cmd), "%c\n", CMD_GET_MOTION);

	if (sendCommand(cmd, res, -1, -1) == false)
	{
		return false;
	}

	float body_curr_az, body_curr_alt;
	float body_setpoint_az, body_setpoint_alt;

	sscanf(res, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,", &body_curr_az,
		   &body_curr_alt, &body_setpoint_az, &body_setpoint_alt,
		   &m_current_azimuth_pos, &m_current_altitude_pos, &m_set_azimuth_pos,
		   &m_set_altitude_pos, &m_azimuth_velocity, &m_altitude_velocity,
		   &m_azimuth_acceleration, &m_altitude_acceleration);

	/*convert the current radiotelescope Az/Alt position into Ra/Dec to share
	 * the info of where the antenna is currently pointing to through INDI*/

	// Convert current Az/Alt → RA/DEC for INDI clients
	double currentRA, currentDEC;
	convertAzAltToRaDec(m_current_azimuth_pos, m_current_altitude_pos,
						currentRA, currentDEC, 0);

	NewRaDec(currentRA, currentDEC);

	TargetNP[0].setValue(m_current_azimuth_pos);
	TargetNP[1].setValue(m_current_altitude_pos);
	TargetNP.setState(IPS_OK);
	TargetNP.apply();  // equivalent of IDSetNumber, pushes to clients

	/* UPDATE DEVICE STATE TEXT*/
	const char* stateStr = "Unknown";
	switch (device_state)
	{
		case DEV_STATE_UNINIT:
			stateStr = "Uninitialised";
			break;
		case DEV_STATE_HOMING:
			stateStr = "Homing";
			break;
		case DEV_STATE_CLOSED:
			stateStr = "Closed/Parked";
			break;
		case DEV_STATE_OPENING:
			stateStr = "Opening";
			break;
		case DEV_STATE_OPEN:
			stateStr = "Open";
			break;
		case DEV_STATE_CLOSING:
			stateStr = "Closing";
			break;
		case DEV_STATE_FAULT:
			stateStr = "FAULT";
			break;
		default:
			stateStr = "Unknown";
			break;
	}

	DeviceStateTP[0].setText(stateStr);
	// Use IPS_ALERT for fault to make it turn red in the GUI
	DeviceStateTP.setState(device_state == DEV_STATE_FAULT ? IPS_ALERT
														   : IPS_OK);
	DeviceStateTP.apply();

	// Current measured values
	MotionStateNP[0].setValue(m_azimuth_velocity);
	MotionStateNP[1].setValue(m_altitude_velocity);
	MotionStateNP[2].setValue(m_azimuth_acceleration);
	MotionStateNP[3].setValue(m_altitude_acceleration);
	MotionStateNP.setState(IPS_OK);
	MotionStateNP.apply();

	// Sync displayed limits from what firmware reported
	MotionLimitsNP[0].setValue(m_max_az_velocity);
	MotionLimitsNP[1].setValue(m_max_alt_velocity);
	MotionLimitsNP[2].setValue(m_max_az_acc);
	MotionLimitsNP[3].setValue(m_max_alt_acc);
	MotionLimitsNP.setState(IPS_OK);
	MotionLimitsNP.apply();

	switch (device_state)
	{
		case DEV_STATE_UNINIT:
			TrackState = SCOPE_IDLE;
			break;

		case DEV_STATE_HOMING:
			TrackState = SCOPE_PARKING;
			break;

		case DEV_STATE_CLOSED:
			TrackState = SCOPE_PARKED;

			ParkSP[0].setState(ISS_OFF);
			ParkSP[1].setState(ISS_ON);
			break;

		case DEV_STATE_OPENING:
			TrackState = SCOPE_SLEWING;
			break;

		case DEV_STATE_OPEN:
			ParkSP[0].setState(ISS_ON);
			ParkSP[1].setState(ISS_OFF);

			/*State of whether manual control or tracking is active*/
			switch (driver_state)
			{
				case DRV_ST_IDLE:
					/*if setpoint and current pos differ, assume slewing*/
					if ((abs(m_current_altitude_pos - m_set_altitude_pos) >
						 1.0f) ||
						(abs(m_current_azimuth_pos - m_set_azimuth_pos) > 1.0f))
					{
						TrackState = SCOPE_SLEWING;
					}
					else
					{
						TrackState = SCOPE_IDLE;
					}

					TrackSatSP[0].setState(ISS_OFF);
					TrackSatSP[1].setState(ISS_ON);
					TrackSatSP.setState(IPS_ALERT);
					TrackSatSP.apply();

					TrackStateSP[0].setState(ISS_OFF);
					TrackStateSP[1].setState(ISS_ON);
					TrackStateSP.setState(IPS_ALERT);
					TrackStateSP.apply();

					break;

				case DRV_ST_SKY_TRACK:
					TrackState = SCOPE_TRACKING;

					TrackSatSP[0].setState(ISS_OFF);
					TrackSatSP[1].setState(ISS_ON);
					TrackSatSP.setState(IPS_ALERT);
					TrackSatSP.apply();

					TrackStateSP[0].setState(ISS_ON);
					TrackStateSP[1].setState(ISS_OFF);
					TrackStateSP.setState(IPS_ALERT);
					TrackStateSP.apply();

					{
						double t_az, t_alt, t_az_rate, t_alt_rate;
						if (computeSkyAzAltRates(m_tracked_ra, m_tracked_dec,
												 0.0, t_az, t_alt, t_az_rate,
												 t_alt_rate))
						{
							if (t_alt < 0.0)
							{
								LOG_WARN(
									"Tracked object below horizon — stopping "
									"sky tracking");
								driver_state = DRV_ST_IDLE;
								snprintf(cmd, sizeof(cmd), "%c%0.4f,%0.4f\n",
										 CMD_GOTO, m_current_azimuth_pos,
										 m_current_altitude_pos);
								sendCommand(cmd);
								break;
							}

							snprintf(cmd, sizeof(cmd),
									 "%c%0.4f,%0.4f,%0.6f,%0.6f\n", CMD_TRACK,
									 t_az, t_alt, t_az_rate, t_alt_rate);
							if (!sendCommand(cmd))
								LOG_ERROR("Failed to send sky track command");
						}
					}
					break;

				case DRV_ST_SAT_TRACK:
					TrackState = SCOPE_TRACKING;

					TrackSatSP[0].setState(ISS_ON);
					TrackSatSP[1].setState(ISS_OFF);
					TrackSatSP.setState(IPS_ALERT);
					TrackSatSP.apply();

					TrackStateSP[0].setState(ISS_OFF);
					TrackStateSP[1].setState(ISS_ON);
					TrackStateSP.setState(IPS_ALERT);
					TrackStateSP.apply();

					{
						double t_az, t_alt, t_az_rate, t_alt_rate;
						if (computeSatAzAltRates(sat_track_sec_offset, t_az,
												 t_alt, t_az_rate, t_alt_rate))
						{
							if (t_alt < 0.0)
							{
								LOG_INFO(
									"Satellite below horizon — stopping "
									"satellite tracking");
								driver_state = DRV_ST_IDLE;
								snprintf(cmd, sizeof(cmd), "%c%0.4f,%0.4f\n",
										 CMD_GOTO, m_current_azimuth_pos,
										 m_current_altitude_pos);
								sendCommand(cmd);
								break;
							}

							snprintf(cmd, sizeof(cmd),
									 "%c%0.4f,%0.4f,%0.6f,%0.6f\n", CMD_TRACK,
									 t_az, t_alt, t_az_rate, t_alt_rate);
							if (!sendCommand(cmd))
								LOG_ERROR(
									"Failed to send satellite track command");
						}
						else
						{
							LOG_ERROR(
								"SGP4 propagation failed — stopping satellite "
								"tracking");
							driver_state = DRV_ST_IDLE;
						}
					}
					break;

				default:
					break;
			}

			break;

		case DEV_STATE_CLOSING:
			TrackState = SCOPE_PARKING;
			break;

		case DEV_STATE_FAULT:
			TrackState = SCOPE_IDLE;
			break;

		default:
			break;
	}

	return true;
}

bool MountDriver::Goto(double RA, double DE)
{
	char cmd[DRIVER_LEN] = {0};
	// char res[DRIVER_LEN] = {0};

	/* convert equatorial coord system to horizontal coord system*/

	if (TrackState == SCOPE_PARKED)
	{
		LOG_ERROR("Please unpark the mount before issuing any goto commands.");
		return false;
	}

	if (m_GeographicLocation.latitude == 0 &&
		m_GeographicLocation.longitude == 0)
	{
		LOG_ERROR("Location not set, cannot compute goto");
		return false;
	}

	/*convert RA DE to AZ EL*/
	double conv_az, conv_alt;
	convertRaDecToAzAlt(RA, DE, conv_az, conv_alt, 0);

	/*Normalize azimuth to [0, 360)*/
	while (conv_az < 0.0) conv_az += 360.0;
	while (conv_az >= 360.0) conv_az -= 360.0;

	/*Clamp altitude to valid body range*/
	if (conv_alt < 0.0) conv_alt = 0.0;
	if (conv_alt > 90.0) conv_alt = 90.0;

	/*Keep this to the value returned by the telescope*/
	// m_set_azimuth_pos = conv_az;
	// m_set_altitude_pos = conv_alt;

	/*prep the packet to send*/
	snprintf(cmd, DRIVER_LEN, "%c%0.4f,%0.4f\n", CMD_GOTO, conv_az, conv_alt);

	// So that the software tracker knows what to follow
	// will re-assign same value if tracking mode is active
	m_tracked_ra = RA;
	m_tracked_dec = DE;

	if (sendCommand(cmd) == false)
	{
		LOG_ERROR("Couldn't send goto command");
		return false;
	}

	char InfoStr[DRIVER_LEN] = {0};
	sprintf(InfoStr,
			"Slewing to AZ: %0.4f - ALT: %0.4f. Set Az: %0.4f - Alt: %0.4f",
			conv_az, conv_alt, m_set_azimuth_pos, m_set_altitude_pos);

	LOGF_INFO("%s", InfoStr);

	return true;
}

bool MountDriver::Sync(double RA, double DE)
{
	double world_az, world_alt;
	convertRaDecToAzAlt(RA, DE, world_az, world_alt, 0.0);

	/* Ring 1 → 2 → 3 → 1 */
	m_sync_point_count++;
	if (m_sync_point_count > 3) m_sync_point_count = 1;

	char cmd[DRIVER_LEN] = {0};
	snprintf(cmd, DRIVER_LEN, "%c%u,%0.4f,%0.4f\n", CMD_SYNC,
			 (unsigned)m_sync_point_count, world_az, world_alt);

	if (!sendCommand(cmd))
	{
		LOG_ERROR("Couldn't send sync command to firmware");
		return false;
	}

	LOGF_INFO("Sync point %u sent — world Az:%.4f Alt:%.4f", m_sync_point_count,
			  world_az, world_alt);

	NewRaDec(RA, DE);

	SyncPointCountNP[0].setValue(m_sync_point_count);
	SyncPointCountNP.setState(IPS_OK);
	SyncPointCountNP.apply();

	return true;
}

bool MountDriver::Park()
{
	char cmd[DRIVER_LEN] = {0};

	snprintf(cmd, DRIVER_LEN, "%c\n", CMD_PARK);

	if (sendCommand(cmd) == false)
	{
		LOG_ERROR("Couldn't send Park/close command");
		return false;
	}

	LOG_INFO("Parking telescope in progress...");
	return true;
}

bool MountDriver::UnPark()
{
	char cmd[DRIVER_LEN] = {0};

	snprintf(cmd, DRIVER_LEN, "%c\n", CMD_UNPARK);

	if (sendCommand(cmd) == false)
	{
		LOG_ERROR("Couldn't send Park/close command");
		return false;
	}

	return true;
}

bool MountDriver::computeSatAzAlt(double& az, double& alt)
{
	if (!m_tle) return false;

	struct timeval tv;
	gettimeofday(&tv, nullptr);
	long millis = (long)(tv.tv_sec * 1000LL + tv.tv_usec / 1000);

	double r[3], v[3];
	m_tle->getRVForDate(millis, r, v);

	if (m_tle->sgp4Error != 0)
	{
		LOGF_ERROR("SGP4 propagation error: %d", m_tle->sgp4Error);
		return false;
	}

	double lat_rad = m_GeographicLocation.latitude * deg2rad;
	double lon_rad = m_GeographicLocation.longitude * deg2rad;

	double jd_now = ln_get_julian_from_sys();
	double gst_hours = ln_get_apparent_sidereal_time(jd_now);
	double gst_rad = gst_hours * (M_PI / 12.0);
	double lst_rad = gst_rad + lon_rad;

	// Observer ECI (km)
	const double re_km = 6378.135;
	double obs_x = re_km * cos(lat_rad) * cos(lst_rad);
	double obs_y = re_km * cos(lat_rad) * sin(lst_rad);
	double obs_z = re_km * sin(lat_rad);

	// Range vector ECI
	double dx = r[0] - obs_x;
	double dy = r[1] - obs_y;
	double dz = r[2] - obs_z;

	// ECI → RA/DEC (radians)
	double range = sqrt(dx * dx + dy * dy + dz * dz);
	if (range < 1.0) return false;

	double dec_rad = asin(dz / range);
	double ra_rad = atan2(dy, dx) - lst_rad + lon_rad;
	// ra_rad is now in the frame of the vernal equinox — normalise
	while (ra_rad < 0) ra_rad += 2.0 * M_PI;
	while (ra_rad > 2 * M_PI) ra_rad -= 2.0 * M_PI;

	// Hand off to libnova for the final RA/DEC → Az/Alt conversion,
	// so the output Az convention exactly matches what Goto() produces.
	ln_equ_posn equ_pos;
	equ_pos.ra = ra_rad / deg2rad;	// libnova wants degrees
	equ_pos.dec = dec_rad / deg2rad;

	ln_lnlat_posn observer;
	observer.lat = m_GeographicLocation.latitude;
	observer.lng = m_GeographicLocation.longitude;

	ln_hrz_posn hrz_pos;
	ln_get_hrz_from_equ(&equ_pos, &observer, jd_now, &hrz_pos);

	az = hrz_pos.az;
	alt = hrz_pos.alt;

	return true;
}

/*Beware, TEME vs TOD(ECI?) frame mismatch unless you keep it considered. SGP4
 * differs from libnova*/
bool MountDriver::computeSatRaDec(double& Ra, double& Dec,
								  double seconds_offset)
{
	if (!m_tle) return false;

	struct timeval tv;
	gettimeofday(&tv, nullptr);

	// /*split double seconds into two ints*/
	int sec_int = static_cast<int>(seconds_offset);
	int usec_int =
		static_cast<int>(std::round((seconds_offset - sec_int) * 1'000'000));

	/*Apply offsets*/
	tv.tv_sec += sec_int;
	tv.tv_usec += usec_int;

	// Carry over microseconds overflow into seconds
	tv.tv_sec += tv.tv_usec / 1'000'000;
	tv.tv_usec = tv.tv_usec % 1'000'000;

	/*SGP4 millis is used for SGP4 propagation, where sat is
	jd now is for sidereal time, where observer is pointing*/

	long millis = (long)(tv.tv_sec * 1000LL + tv.tv_usec / 1000);

	double r[3], v[3];
	m_tle->getRVForDate(millis, r, v);

	if (m_tle->sgp4Error != 0)
	{
		LOGF_ERROR("SGP4 propagation error: %d", m_tle->sgp4Error);
		return false;
	}

	/*Need to convert TEME to TOD*/
	double lat_rad = m_GeographicLocation.latitude * deg2rad;
	double lon_rad = (m_GeographicLocation.longitude) * deg2rad;

	double jd_offset = getOffsetJulian(seconds_offset);
	double gst_hours = ln_get_apparent_sidereal_time(jd_offset);
	double gst_rad = gst_hours * (M_PI / 12.0);
	double lst_rad = gst_rad + lon_rad;

	// Observer ECI (km)
	const double re_km = 6378.137;
	double obs_x = re_km * cos(lat_rad) * cos(lst_rad);
	double obs_y = re_km * cos(lat_rad) * sin(lst_rad);
	double obs_z = re_km * sin(lat_rad);

	// Range vector ECI
	double dx = r[0] - obs_x;
	double dy = r[1] - obs_y;
	double dz = r[2] - obs_z;

	// ECI → RA/DEC (radians)
	double range = sqrt(dx * dx + dy * dy + dz * dz);
	if (range < 1.0) return false;

	double ra_rad = atan2(dy, dx);
	if (ra_rad < 0) ra_rad += 2.0 * M_PI;
	double dec_rad = asin(dz / range);

	Ra = ra_rad * 12.0 / M_PI;	   // hours, 0..24
	Dec = dec_rad * 180.0 / M_PI;  // degrees

	return true;
}

/*doesn't apply the hardware position offset, just computes the az/alt from
 * given ra/dec*/
bool MountDriver::convertRaDecToAzAlt(double Ra, double Dec, double& az,
									  double& alt, double seconds_offset)
{
	if (m_GeographicLocation.latitude == 0 &&
		m_GeographicLocation.longitude == 0)
	{
		LOG_ERROR("Location not set, cannot compute Az/Alt from equ coords");
		return false;
	}

	/*convert RA DE to AZ EL*/
	ln_equ_posn equatorial_pos;

	equatorial_pos.ra = Ra * 360 / 24.0;
	equatorial_pos.dec = Dec;

	/*filled by libnova*/
	ln_hrz_posn horizontal_pos;

	ln_lnlat_posn observer_pos;
	/*Ecliptical (celestial) lat/long. Angles in degrees, east positive, west
	 * negative*/
	observer_pos.lat = m_GeographicLocation.latitude;
	observer_pos.lng = m_GeographicLocation.longitude;

	/*get julian date with offset applied*/
	double JD = getOffsetJulian(seconds_offset);

	/*convert horizontal coord to equatorial*/
	ln_get_hrz_from_equ(&equatorial_pos, &observer_pos, JD, &horizontal_pos);

	/* libnova returns az in the South=0 (astronomical) convention.
	 * The firmware and all real-world use North=0 (compass) convention.
	 * (az + 180) % 360 converts between them — the formula is self-inverse. */
	az = fmod(horizontal_pos.az + 180.0, 360.0);
	alt = horizontal_pos.alt;

	char InfoStr[DRIVER_LEN] = {0};

	sprintf(InfoStr, "Converted Ra: %0.4f Dec: %0.4f to Az: %0.4f. Alt: %0.4f ",
			Ra, Dec, az, horizontal_pos.alt);

	// LOGF_INFO("%s", InfoStr);

	return true;
}

bool MountDriver::convertAzAltToRaDec(double az, double alt, double& Ra,
									  double& Dec, double seconds_offset)
{
	// Convert current Az/Alt → RA/DEC for INDI clients
	ln_hrz_posn hrz_pos;
	/* Firmware / caller uses North=0 (compass). libnova expects South=0.
	 * Same (az + 180) % 360 conversion as in convertRaDecToAzAlt. */
	hrz_pos.az = fmod(az + 180.0, 360.0);
	hrz_pos.alt = alt;

	ln_lnlat_posn observer;
	observer.lat = m_GeographicLocation.latitude;
	observer.lng = m_GeographicLocation.longitude;

	/*get possibly offset julian date*/
	double JD = getOffsetJulian(seconds_offset);

	ln_equ_posn equ_pos;
	ln_get_equ_from_hrz(&hrz_pos, &observer, JD, &equ_pos);

	Ra = equ_pos.ra * 24.0 / 360.0;	 // degrees → hours
	Dec = equ_pos.dec;
	return true;
}

bool MountDriver::computeSkyAzAltRates(double ra, double dec, double sec_offset,
									   double& az, double& alt, double& az_rate,
									   double& alt_rate)
{
	double az1, alt1, az2, alt2;
	if (!convertRaDecToAzAlt(ra, dec, az1, alt1, sec_offset)) return false;
	if (!convertRaDecToAzAlt(ra, dec, az2, alt2, sec_offset + 1.0))
		return false;

	az = az1;
	alt = alt1;

	/* Azimuth wrap correction: handles targets crossing 0°/360° */
	double daz = az2 - az1;
	if (daz > 180.0) daz -= 360.0;
	if (daz < -180.0) daz += 360.0;

	az_rate = daz; /* deg/s — delta over exactly 1 s */
	alt_rate = alt2 - alt1;
	return true;
}

bool MountDriver::computeSatAzAltRates(double sec_offset, double& az,
									   double& alt, double& az_rate,
									   double& alt_rate)
{
	double ra1, dec1, ra2, dec2;
	if (!computeSatRaDec(ra1, dec1, sec_offset)) return false;
	if (!computeSatRaDec(ra2, dec2, sec_offset + 1.0)) return false;

	double az1, alt1, az2, alt2;
	if (!convertRaDecToAzAlt(ra1, dec1, az1, alt1, sec_offset)) return false;
	if (!convertRaDecToAzAlt(ra2, dec2, az2, alt2, sec_offset + 1.0))
		return false;

	az = az1;
	alt = alt1;

	double daz = az2 - az1;
	if (daz > 180.0) daz -= 360.0;
	if (daz < -180.0) daz += 360.0;

	az_rate = daz;
	alt_rate = alt2 - alt1;
	return true;
}

/*custom julian day from system with offset in seconds*/
double MountDriver::getOffsetJulian(double offset_seconds)
{
	double JD;
	struct ln_date date;

	struct tm* gmt;
	struct timeval tv;
	struct timezone tz;

	/* get current time with microseconds precission*/
	gettimeofday(&tv, &tz);

	/*spilt seconds in double into seconds and microseconds split*/
	int sec_int = static_cast<int>(offset_seconds);
	int usec_int =
		static_cast<int>(std::round((offset_seconds - sec_int) * 1'000'000));

	/*apply offset*/
	tv.tv_sec += sec_int;
	tv.tv_usec += usec_int;

	// Carry over microseconds overflow into seconds
	tv.tv_sec += tv.tv_usec / 1'000'000;
	tv.tv_usec = tv.tv_usec % 1'000'000;

	/* convert to UTC time representation */
	gmt = gmtime(&tv.tv_sec);

	/* fill in date struct */
	date.seconds = gmt->tm_sec + ((double)tv.tv_usec / 1000000);
	date.minutes = gmt->tm_min;
	date.hours = gmt->tm_hour;
	date.days = gmt->tm_mday;
	date.months = gmt->tm_mon + 1;
	date.years = gmt->tm_year + 1900;

	/*so far from get date from sys*/

	/*make system time into julian day*/
	JD = ln_get_julian_day(&date);

	return JD;
}

bool MountDriver::ISNewNumber(const char* dev, const char* name,
							  double values[], char* names[], int n)
{
	if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
	{
		if (MotionLimitsNP.isNameMatch(name))
		{
			MotionLimitsNP.update(values, names, n);

			char cmd[DRIVER_LEN] = {0};

			// Send velocity command
			snprintf(cmd, DRIVER_LEN, "%c%0.4f,%0.4f\n", CMD_SET_VEL,
					 MotionLimitsNP[0].getValue(),
					 MotionLimitsNP[1].getValue());
			sendCommand(cmd);

			usleep(10000);

			// Send acceleration command
			snprintf(cmd, DRIVER_LEN, "%c%0.4f,%0.4f\n", CMD_SET_ACC,
					 MotionLimitsNP[2].getValue(),
					 MotionLimitsNP[3].getValue());
			sendCommand(cmd);

			MotionLimitsNP.setState(IPS_OK);
			MotionLimitsNP.apply();
			return true;
		}
		if (SatTrackLeadSecondsNP.isNameMatch(name))
		{
			SatTrackLeadSecondsNP.update(values, names, n);
			sat_track_sec_offset = SatTrackLeadSecondsNP[0].getValue();
			SatTrackLeadSecondsNP.setState(IPS_OK);
			SatTrackLeadSecondsNP.apply();
		}
	}
	return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool MountDriver::ISNewText(const char* dev, const char* name, char* texts[],
							char* names[], int n)
{
	if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
	{
		if (strcmp(name, "SAT_TLE_TEXT") == 0)
		{
			// TLEtoTrackTP is inherited from INDI::Telescope
			TLEtoTrackTP.update(texts, names, n);

			// TLEtoTrackTP[0] is the full 3-line TLE as a single string
			// Split on newlines to get the two lines the TLE class needs
			std::string full_tle = TLEtoTrackTP[0].getText();
			std::istringstream ss(full_tle);
			std::string name_line, line1, line2;
			std::getline(ss, name_line);
			std::getline(ss, line1);
			std::getline(ss, line2);

			if (line1.empty() || line2.empty())
			{
				LOG_ERROR("TLE format invalid — expected 3 lines");
				TLEtoTrackTP.setState(IPS_ALERT);
				TLEtoTrackTP.apply();
				return false;
			}

			char l1[70], l2[70];
			strncpy(l1, line1.c_str(), 69);
			l1[69] = '\0';
			strncpy(l2, line2.c_str(), 69);
			l2[69] = '\0';

			m_tle_name = name_line;
			m_tle = std::make_unique<TLE>(l1, l2);

			if (m_tle->sgp4Error != 0)
			{
				LOGF_ERROR("TLE init failed (sgp4 error %d)", m_tle->sgp4Error);
				TLEtoTrackTP.setState(IPS_ALERT);
				TLEtoTrackTP.apply();
				return false;
			}

			LOGF_INFO("TLE loaded: %s", m_tle_name.c_str());
			TLEtoTrackTP.setState(IPS_OK);
			TLEtoTrackTP.apply();
			return true;
		}
	}
	return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool MountDriver::ISNewSwitch(const char* dev, const char* name,
							  ISState* states, char* names[], int n)
{
	if (LNBPowerSP.isNameMatch(name))
	{
		LNBPowerSP.update(states, names, n);
		uint8_t pol = 0;
		if (LNBPowerSP[1].getState() == ISS_ON)
			pol = 1;
		else if (LNBPowerSP[2].getState() == ISS_ON)
			pol = 2;
		uint8_t ifsel = (LNBLocalOscSP[1].getState() == ISS_ON) ? 1 : 0;
		LNBPowerSP.setState(sendLNBCommand(pol, ifsel) ? IPS_OK : IPS_ALERT);
		LNBPowerSP.apply();
		return true;
	}

	if (LNBLocalOscSP.isNameMatch(name))
	{
		LNBLocalOscSP.update(states, names, n);
		uint8_t pol = 0;
		if (LNBPowerSP[1].getState() == ISS_ON)
			pol = 1;
		else if (LNBPowerSP[2].getState() == ISS_ON)
			pol = 2;
		uint8_t ifsel = (LNBLocalOscSP[1].getState() == ISS_ON) ? 1 : 0;
		LNBLocalOscSP.setState(sendLNBCommand(pol, ifsel) ? IPS_OK : IPS_ALERT);
		LNBLocalOscSP.apply();
		return true;
	}

	if (TrackSatSP.isNameMatch(name))
	{
		// refuse to enable without a valid loaded TLE
		if (!m_tle || m_tle->sgp4Error != 0)
		{
			LOG_ERROR("Load a valid TLE before enabling satellite tracking");
			TrackSatSP.setState(IPS_ALERT);
			TrackSatSP.apply();
			return true;
		}

		TrackSatSP.update(states, names, n);

		if (TrackSatSP[0].getState() == ISS_ON)
		{
			if (driver_state == DRV_ST_IDLE)
			{
				/* Refuse to start if satellite is currently below the horizon
				 */
				double chk_az, chk_alt, chk_azr, chk_altr;
				if (!computeSatAzAltRates(sat_track_sec_offset, chk_az, chk_alt,
										  chk_azr, chk_altr) ||
					chk_alt < 0.0)
				{
					LOG_ERROR("Satellite not currently visible above horizon");
					TrackSatSP.setState(IPS_ALERT);
					TrackSatSP.apply();
					return true;
				}

				LOG_INFO("Satellite tracking enabled");
				driver_state = DRV_ST_SAT_TRACK;
				TrackSatSP.setState(IPS_BUSY);
				TrackSatSP.apply();
			}
		}
		else
		{
			if (driver_state == DRV_ST_SAT_TRACK)
			{
				LOG_INFO("Satellite tracking disabed");
				driver_state = DRV_ST_IDLE;
				TrackSatSP.setState(IPS_IDLE);
				TrackSatSP.apply();
			}
		}

		LOG_INFO("Satellite track set");
		return true;
	}

	/*If home run button things happened*/
	if (HomingRunSP.isNameMatch(name))
	{
		char cmd[DRIVER_LEN];
		if (HomingRunSP[0].getState() == ISS_ON)
		{
			snprintf(cmd, DRIVER_LEN, "%c\n", CMD_HOME);
			sendCommand(cmd);
			LOG_INFO("Sent homing CMD.");
			HomingRunSP[0].setState(ISS_OFF);
		}
		else if (HomingRunSP[1].getState() == ISS_ON)
		{
			snprintf(cmd, DRIVER_LEN, "%c\n", CMD_HOME);
			sendCommand(cmd);
			LOG_INFO("Sent UNSAFE homing CMD.");
			HomingRunSP[1].setState(ISS_OFF);
		}
	}

	if (ResetButtonSP.isNameMatch(name))
	{
		if (ResetButtonSP[0].getState() == ISS_ON)
		{
			char cmd[DRIVER_LEN];
			snprintf(cmd, DRIVER_LEN, "%c\n", CMD_HW_RESET);
			sendCommand(cmd);
			LOG_INFO("Sent hardware reset to R3.");
			ResetButtonSP[0].setState(ISS_OFF);
			ResetButtonSP.apply();
		}
	}

	if (ResetSyncSP.isNameMatch(name))
	{
		if (ResetSyncSP[0].getState() == ISS_ON)
		{
			char cmd[DRIVER_LEN] = {0};
			snprintf(cmd, DRIVER_LEN, "%c0,0.0000,0.0000\n", CMD_SYNC);
			if (!sendCommand(cmd))
			{
				LOG_ERROR("Failed to reset firmware sync/pointing model");
				ResetSyncSP.setState(IPS_ALERT);
			}
			else
			{
				m_sync_point_count = 0;
				SyncPointCountNP[0].setValue(0);
				SyncPointCountNP.setState(IPS_OK);
				SyncPointCountNP.apply();
				LOG_INFO("Firmware pointing model reset (w0 sent)");
				ResetSyncSP.setState(IPS_OK);
			}
			ResetSyncSP[0].setState(ISS_OFF);
			ResetSyncSP.apply();
		}
		return true;
	}

	return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool MountDriver::Abort()
{
	if (TrackState == SCOPE_PARKED)
	{
		LOG_ERROR("Can't abort in parked mode.");
		return true;
	}

	driver_state = DRV_ST_IDLE;

	/* Send a position-mode GOTO at the current body position.
	 * This exits the firmware's CV tracking mode (track_mode → 0)
	 * and holds the dish where it is. */
	char cmd[DRIVER_LEN] = {0};
	snprintf(cmd, DRIVER_LEN, "%c%0.4f,%0.4f\n", CMD_GOTO,
			 m_current_azimuth_pos, m_current_altitude_pos);

	if (!sendCommand(cmd))
	{
		LOG_ERROR("Failed to send abort GOTO");
		return false;
	}

	LOG_INFO("Abort: tracking stopped, hardware in position mode");
	return true;
}

/*Antenna pitching - North up, south down. */
bool MountDriver::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
	if (TrackState == SCOPE_PARKED)
	{
		LOG_ERROR(
			"Please unpark the mount before issuing any motion commands.");
		return true;
	}

	/*just increment at initial click, no hold-down ops*/
	if (command == MOTION_STOP) return true;

	/* Manual jog exits any active tracking mode */
	driver_state = DRV_ST_IDLE;

	/*Move the telescope setpoint around */
	/*send goto by incrementing/decrementing where dish is*/
	double conv_RA, conv_DEC;

	if (dir == DIRECTION_NORTH)
	{
		if ((m_set_altitude_pos + 1.0f) <= 75.0f)
		{
			convertAzAltToRaDec(m_set_azimuth_pos, (m_set_altitude_pos + 1.0f),
								conv_RA, conv_DEC, 0);
			Goto(conv_RA, conv_DEC);
		}
	}
	else
	{
		if ((m_set_altitude_pos - 1.0f) >= 0.0f)
		{
			convertAzAltToRaDec(m_set_azimuth_pos, (m_set_altitude_pos - 1.0f),
								conv_RA, conv_DEC, 0);
			Goto(conv_RA, conv_DEC);
		}
	}

	return true;
	// char cmd[DRIVER_LEN] = {0};
	// float new_pos = 0.0f;

	// /*+1|-1 degrees*/
	// if (dir > 0)
	// 	new_pos = m_current_altitude_pos + 1.0f;
	// else
	// 	new_pos = m_current_altitude_pos - 1.0f;

	// sprintf(cmd, "b%0.3f\n", new_pos);

	// if (sendCommand(cmd))
	// {
	// 	TrackState = SCOPE_SLEWING;
	// 	return true;
	// }
	// else
	// {
	// 	return false;
	// }
}

/*West left, east right.*/
bool MountDriver::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
	if (TrackState == SCOPE_PARKED)
	{
		LOG_ERROR(
			"Please unpark the mount before issuing any motion commands.");
		return true;
	}
	/*just increment at initial click, no hold-down ops*/
	if (command == MOTION_STOP) return true;

	/* Manual jog exits any active tracking mode */
	driver_state = DRV_ST_IDLE;

	/*Move the telescope setpoint around */
	/*send goto by incrementing/decrementing where dish is*/
	double conv_RA, conv_DEC, modulod_az;

	if (dir == DIRECTION_EAST)
	{
		modulod_az = m_set_azimuth_pos + 1.0f;
		if (modulod_az > 360.0f) modulod_az -= 360.0f;
	}
	else
	{
		modulod_az = m_set_azimuth_pos - 1.0f;
		if (modulod_az < 0.0f) modulod_az += 360.0f;
	}

	convertAzAltToRaDec(modulod_az, m_set_altitude_pos, conv_RA, conv_DEC, 0);
	Goto(conv_RA, conv_DEC);

	return true;
}

IPState MountDriver::GuideNorth(uint32_t ms)
{
	// Implement here the actual calls to do the motion requested
	INDI_UNUSED(ms);
	return IPS_BUSY;
}

IPState MountDriver::GuideSouth(uint32_t ms)
{
	// Implement here the actual calls to do the motion requested
	INDI_UNUSED(ms);
	return IPS_BUSY;
}

IPState MountDriver::GuideEast(uint32_t ms)
{
	// Implement here the actual calls to do the motion requested
	INDI_UNUSED(ms);
	return IPS_BUSY;
}

IPState MountDriver::GuideWest(uint32_t ms)
{
	// Implement here the actual calls to do the motion requested
	INDI_UNUSED(ms);
	return IPS_BUSY;
}

bool MountDriver::updateLocation(double latitude, double longitude,
								 double elevation)
{
	// INDI_UNUSED(elevation);
	// JM: INDI Longitude is 0 to 360 increasing EAST. libnova East is
	// Positive, West is negative
	m_GeographicLocation.longitude = longitude;

	if (m_GeographicLocation.longitude > 180)
		m_GeographicLocation.longitude -= 360;

	m_GeographicLocation.latitude = latitude;

	m_GeographicLocation.elevation = elevation;

	// Implement here the actual calls to the controller to set the location
	// if supported.

	// Inform the client that location was updated if all goes well
	LOGF_INFO("Location updated: Longitude (%g) Latitude (%g)",
			  m_GeographicLocation.longitude, m_GeographicLocation.latitude);

	return true;
}

bool MountDriver::SetCurrentPark()
{
	// Depending on the parking type defined initially (PARK_RA_DEC or
	// PARK_AZ_ALT...etc) set the current position AS the parking position.

	// Assumg PARK_AZ_ALT, we need to do something like this:

	// SetAxis1Park(getCurrentAz());
	// SetAxis2Park(getCurrentAlt());

	// Or if currentAz, currentAlt are defined as variables in our driver,
	// then SetAxis1Park(currentAz); SetAxis2Park(currentAlt);

	return true;
}

bool MountDriver::SetDefaultPark()
{
	// For RA_DE park, we can use something like this:

	// By default set RA to HA
	// SetAxis1Park(get_local_sidereal_time(LocationNP[LOCATION_LONGITUDE].value));
	// Set DEC to 90 or -90 depending on the hemisphere
	// SetAxis2Park((LocationNP[LOCATION_LATITUDE].value > 0) ? 90 : -90);

	// For Az/Alt, we can use something like this:

	// Az = 0
	// SetAxis1Park(0);
	// Alt = 0
	// SetAxis2Park(0);

	return true;
}

bool MountDriver::SetTrackMode(uint8_t mode)
{
	// Sidereal/Lunar/Solar..etc

	// Send actual command here to device
	INDI_UNUSED(mode);
	return true;
}

bool MountDriver::SetTrackEnabled(bool enabled)
{
	/*Enable tracking only if driver state is "idle"*/

	if (enabled)
	{
		if (driver_state != DRV_ST_IDLE)
		{
			LOG_ERROR("Can't enable sky tracking. Driver needs to be IDLE.");
			return true;
		}

		driver_state = DRV_ST_SKY_TRACK;

		LOG_INFO("Tracking enabled — will update Az/Alt each poll cycle");
		m_tracked_ra = EqNP[AXIS_RA].getValue();
		m_tracked_dec = EqNP[AXIS_DE].getValue();
	}
	else
	{
		if (driver_state == DRV_ST_SKY_TRACK ||
			driver_state == DRV_ST_SAT_TRACK)
		{
			driver_state = DRV_ST_IDLE;
			LOG_INFO("Tracking disabled");

			/* Exit firmware CV mode by issuing a position-mode GOTO
			 * at the current body position. */
			char stop_cmd[DRIVER_LEN] = {0};
			snprintf(stop_cmd, sizeof(stop_cmd), "%c%0.4f,%0.4f\n", CMD_GOTO,
					 m_current_azimuth_pos, m_current_altitude_pos);
			sendCommand(stop_cmd);
		}
		else
		{
			LOG_WARN("Tracking disabled, but no tracking state was active");
		}
	}

	char cmd[DRIVER_LEN] = {0};
	snprintf(cmd, sizeof(cmd), "%c7\n", CMD_CHIRP);
	if (sendCommand(cmd, nullptr, -1, -1) == false)
	{
		LOG_INFO("Couldn't chirp");
		return false;
	}

	return true;
}

bool MountDriver::SetTrackRate(double raRate, double deRate)
{
	// Send actual command here to device
	INDI_UNUSED(raRate);
	INDI_UNUSED(deRate);
	return true;
}

bool MountDriver::sendCommand(const char* cmd, char* res, int cmd_len,
							  int res_len)
{
	int nbytes_written = 0, nbytes_read = 0, rc = -1;

	tcflush(PortFD, TCIOFLUSH);

	if (cmd_len > 0)
	{
		char hex_cmd[DRIVER_LEN * 3] = {0};
		hexDump(hex_cmd, cmd, cmd_len);
		LOGF_DEBUG("CMD <%s>", hex_cmd);
		rc = tty_write(PortFD, cmd, cmd_len, &nbytes_written);
	}
	else
	{
		LOGF_DEBUG("CMD <%s>", cmd);
		rc = tty_write_string(PortFD, cmd, &nbytes_written);
	}

	if (rc != TTY_OK)
	{
		char errstr[MAXRBUF] = {0};
		tty_error_msg(rc, errstr, MAXRBUF);
		LOGF_ERROR("Serial write error: %s.", errstr);
		return false;
	}

	if (res == nullptr) return true;

	if (res_len > 0)
		rc = tty_read(PortFD, res, res_len, DRIVER_TIMEOUT, &nbytes_read);
	else
		rc = tty_nread_section(PortFD, res, DRIVER_LEN, DRIVER_STOP_CHAR,
							   DRIVER_TIMEOUT, &nbytes_read);

	if (rc != TTY_OK)
	{
		char errstr[MAXRBUF] = {0};
		tty_error_msg(rc, errstr, MAXRBUF);
		LOGF_ERROR("Serial read error: %s.", errstr);
		return false;
	}

	if (res_len > 0)
	{
		char hex_res[DRIVER_LEN * 3] = {0};
		hexDump(hex_res, res, res_len);
		LOGF_DEBUG("RES <%s>", hex_res);
	}
	else
	{
		LOGF_DEBUG("RES <%s>", res);
	}

	tcflush(PortFD, TCIOFLUSH);

	return true;
}

bool MountDriver::sendLNBCommand(uint8_t pol, uint8_t ifsel)
{
	char cmd[DRIVER_LEN] = {0};
	snprintf(cmd, DRIVER_LEN, "%c%u,%u\n", CMD_LNBSET, pol, ifsel);
	return sendCommand(cmd);
}

void MountDriver::hexDump(char* buf, const char* data, int size)
{
	for (int i = 0; i < size; i++)
		sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

	if (size > 0) buf[3 * size - 1] = '\0';
}

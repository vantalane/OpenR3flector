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

#pragma once

// #include "connectionplugins/connectionserial.h"
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>

#include "TLE.h"
#include "indiguiderinterface.h"
#include "indipropertynumber.h"
#include "indipropertyswitch.h"
#include "indipropertytext.h"
#include "inditelescope.h"

/**
 * @brief The MountDriver class provides a simple example for development of a
 * new mount driver. Modify the driver to fit your needs.
 *
 * It supports the following features:
 * + Sidereal and Custom Tracking rates.
 * + Goto & Sync
 * + NWSE Hand controller direction key slew.
 * + Tracking On/Off.
 * + Parking & Unparking with custom parking positions.
 * + Setting Time & Location.
 *
 * On startup and by default the mount shall point to the celestial pole.
 *
 * @author Jasem Mutlaq
 */

class MountDriver : public INDI::Telescope, public INDI::GuiderInterface
{
   public:
	MountDriver();

	virtual const char* getDefaultName() override;
	virtual bool initProperties() override;
	virtual bool updateProperties() override;

	virtual bool ISNewNumber(const char* dev, const char* name, double values[],
							 char* names[], int n) override;
	virtual bool ISNewText(const char* dev, const char* name, char* texts[],
						   char* names[], int n) override;
	virtual bool ISNewSwitch(const char* dev, const char* name, ISState* states,
							 char* names[], int n) override;

   protected:
	///////////////////////////////////////////////////////////////////////////////
	/// Communication Commands
	///////////////////////////////////////////////////////////////////////////////
	/**
	 * @brief Handshake Attempt communication with the mount.
	 * @return true if successful, false otherwise.
	 */
	virtual bool Handshake() override;

	/**
	 * @brief ReadScopeStatus Query the mount status, coordinate, any status
	 * indicators, pier side..etc.
	 * @return True if query is successful, false otherwise.
	 */
	virtual bool ReadScopeStatus() override;

	///////////////////////////////////////////////////////////////////////////////
	/// Motions commands.
	///////////////////////////////////////////////////////////////////////////////

	/**
	 * @brief MoveNS Start or Stop motion in the North/South DEC Axis.
	 * @param dir Direction
	 * @param command Start or Stop
	 * @return true if successful, false otherwise.
	 */
	virtual bool MoveNS(INDI_DIR_NS dir,
						TelescopeMotionCommand command) override;

	/**
	 * @brief MoveWE Start or Stop motion in the East/West RA Axis.
	 * @param dir Direction
	 * @param command Start or Stop
	 * @return true if successful, false otherwise.
	 */
	virtual bool MoveWE(INDI_DIR_WE dir,
						TelescopeMotionCommand command) override;

	/**
	 * @brief Abort Abort all motion. If tracking, stop it.
	 * @return True if successful, false otherwise.
	 */
	virtual bool Abort() override;

	///////////////////////////////////////////////////////////////////////////////
	/// Pulse Guiding Commands
	///////////////////////////////////////////////////////////////////////////////
	virtual IPState GuideNorth(uint32_t ms) override;
	virtual IPState GuideSouth(uint32_t ms) override;
	virtual IPState GuideEast(uint32_t ms) override;
	virtual IPState GuideWest(uint32_t ms) override;

	///////////////////////////////////////////////////////////////////////////////
	/// Tracking Commands
	///////////////////////////////////////////////////////////////////////////////
	virtual bool SetTrackMode(uint8_t mode) override;
	virtual bool SetTrackEnabled(bool enabled) override;
	virtual bool SetTrackRate(double raRate, double deRate) override;

	///////////////////////////////////////////////////////////////////////////////
	/// GOTO & Sync commands
	///////////////////////////////////////////////////////////////////////////////
	virtual bool Goto(double RA, double DE) override;
	virtual bool Sync(double RA, double DE) override;

	///////////////////////////////////////////////////////////////////////////////
	/// Time, Date & Location commands.
	///////////////////////////////////////////////////////////////////////////////
	virtual bool updateLocation(double latitude, double longitude,
								double elevation) override;

	///////////////////////////////////////////////////////////////////////////////
	/// Parking commands
	///////////////////////////////////////////////////////////////////////////////
	virtual bool Park() override;
	virtual bool UnPark() override;
	virtual bool SetCurrentPark() override;
	virtual bool SetDefaultPark() override;

	///////////////////////////////////////////////////////////////////////////////
	/// Utility Functions
	///////////////////////////////////////////////////////////////////////////////
	/**
	 * @brief sendCommand Send a string command to device.
	 * @param cmd Command to be sent. Can be either NULL TERMINATED or just byte
	 * buffer.
	 * @param res If not nullptr, the function will wait for a response from the
	 * device. If nullptr, it returns true immediately after the command is
	 * successfully sent.
	 * @param cmd_len if -1, it is assumed that the @a cmd is a null-terminated
	 * string. Otherwise, it would write @a cmd_len bytes from the @a cmd
	 * buffer.
	 * @param res_len if -1 and if @a res is not nullptr, the function will read
	 * until it detects the default delimiter DRIVER_STOP_CHAR up to DRIVER_LEN
	 * length. Otherwise, the function will read @a res_len from the device and
	 * store it in @a res.
	 * @return True if successful, false otherwise.
	 */
	bool sendCommand(const char* cmd, char* res = nullptr, int cmd_len = -1,
					 int res_len = -1);

	/**
	 * @brief hexDump Helper function to print non-string commands to the logger
	 * so it is easier to debug
	 * @param buf buffer to format the command into hex strings.
	 * @param data the command
	 * @param size length of the command in bytes.
	 * @note This is called internally by sendCommand, no need to call it
	 * directly.
	 */
	void hexDump(char* buf, const char* data, int size);
	bool sendLNBCommand(uint8_t pol, uint8_t ifsel);
	bool computeSatAzAlt(double& az, double& alt);
	bool computeSatRaDec(double& Ra, double& Dec, double seconds_offset);
	bool convertRaDecToAzAlt(double Ra, double Dec, double& az, double& alt,
							 double seconds_offset);
	bool convertAzAltToRaDec(double az, double alt, double& Ra, double& Dec,
							 double seconds_offset);

	double getOffsetJulian(double offset_seconds);

	bool computeSkyAzAltRates(double ra, double dec, double sec_offset,
							  double& az, double& alt, double& az_rate,
							  double& alt_rate);
	bool computeSatAzAltRates(double sec_offset, double& az, double& alt,
							  double& az_rate, double& alt_rate);

   private:
	// Mutex for thread safety
	std::mutex m_Mutex;

	INDI::IGeographicCoordinates m_GeographicLocation{0, 0, 0};

	/*THESE VALUES ARE KEPT IN SYNC BY DRIVER POLLING*/

	/*setpoint position - READONLY WORLD FRAME OF DEVICE*/
	float m_set_azimuth_pos{0};
	float m_set_altitude_pos{0};
	/*current position*/
	float m_current_azimuth_pos{0};
	float m_current_altitude_pos{0};

	/*current axis velocities*/
	float m_azimuth_velocity{0};
	float m_altitude_velocity{0};
	/*current axis accelerations*/
	float m_azimuth_acceleration{0};
	float m_altitude_acceleration{0};

	/*configured axis maxI accelerations in deg per sec^2*/
	float m_max_az_acc{0};
	float m_max_alt_acc{0};
	/*configured axis max velocities in deg per sec*/
	float m_max_az_velocity{0};
	float m_max_alt_velocity{0};

	uint8_t endstop_yaw{0};
	uint8_t endstop_pitch{0};

	/* Driver-side state of "mount". */

	/*Meant to be a unified state for handling what things are done.*/
	typedef enum : uint8_t
	{
		DRV_ST_IDLE,	   // SLEWING etc
		DRV_ST_SKY_TRACK,  // SCOPE_TRACKING
		DRV_ST_SAT_TRACK,  // SCOPE_TRACKING
	} DriverState_t;

	DriverState_t driver_state{DRV_ST_IDLE};

	/*copied over from the HW .h files*/
	typedef enum : uint8_t
	{
		DEV_STATE_UNINIT = 0,
		DEV_STATE_HOMING,
		DEV_STATE_CLOSED,
		DEV_STATE_OPENING,
		DEV_STATE_OPEN, /*normal operation state*/
		DEV_STATE_CLOSING,
		DEV_STATE_FAULT,
	} DeviceState_t;

	DeviceState_t device_state{DEV_STATE_UNINIT};

	typedef enum : char
	{
		CMD_HOME = 'h',		   /*perform homing calib*/
		CMD_HOME_UNSAFE = 'u', /*althoming calib*/
		CMD_GOTO = 'g',		   /*goto AZ/ALT pos*/
		CMD_SYNC = 'w',		   /*sync current pos with given pos*/
		CMD_TRACK = 't',	   /*TBD*/
		CMD_MOVE = 'b',		   /*Move in a give direction*/
		CMD_PARK = 'c',
		CMD_UNPARK = 'o',
		CMD_ABORT = 'q',	  /*Stop current movement to setpos*/
		CMD_ESTOP = 'e',	  /*Immediate HALT and enter fault mode*/
		CMD_GET_STATUS = 's', /*get status of device*/
		CMD_GET_MOTION = 'm',
		CMD_SET_ACC = 'a',	/*configure acceleration of axes*/
		CMD_SET_VEL = 'v',	/*configure velocity of axes*/
		CMD_HW_RESET = 'r', /*Resets the hardware*/
		CMD_CHIRP = 'j',
		CMD_DEBUG = 'd',
		CMD_LNBSET = 'p',
		CMD_PLAN = 'l',

	} uart_commands_t;

	/*TODO,combine this with satellite tracking operation*/
	double m_tracked_ra{0};
	double m_tracked_dec{0};

	double m_sat_ra{0.0};
	double m_sat_dec{0.0};

	double m_track_last_update_ra{0};
	double m_track_last_update_dec{0};

	/* Sync point counter — auto-increments 1→2→3→1 on each Sync() call */
	uint8_t m_sync_point_count{0};

	/*display device state*/
	INDI::PropertyText DeviceStateTP{1};
	/*HW Reset*/
	INDI::PropertySwitch ResetButtonSP{1};
	/*Resets all firmware sync/pointing-model points (sends w0,0,0)*/
	INDI::PropertySwitch ResetSyncSP{1};
	/*Read-only display of how many sync points are loaded in firmware*/
	INDI::PropertyNumber SyncPointCountNP{1};
	/*Run safe/unsafe homing procedure*/
	INDI::PropertySwitch HomingRunSP{2};

	/*Satellite tracking data*/

	// INDI::PropertyText TLEtoTrackTP{1};
	INDI::PropertyNumber MotionStateNP{4};	 // current vel/accel readback
	INDI::PropertyNumber MotionLimitsNP{4};	 // configurable max vel/accel

	// INDI::PropertyText SatelliteTLETP{3};  // name, line1, line2
	// INDI::PropertySwitch SatelliteTrackSP{2};  // Enable / Disable
	INDI::PropertyNumber SatTrackLeadSecondsNP{1};
	double sat_track_sec_offset{0};

	/*LNB control*/
	INDI::PropertySwitch LNBPowerSP{3};		// Off / Vertical / Horizontal
	INDI::PropertySwitch LNBLocalOscSP{2};	// Low (9.75 GHz) / High (10.60 GHz)

	/*Pointing-model calibration offsets (read from firmware status)*/
	INDI::PropertyNumber CalibOffsetsNP{5};

	std::unique_ptr<TLE> m_tle;
	std::string m_tle_name;
	std::string m_tle_line1;
	std::string m_tle_line2;

	/////////////////////////////////////////////////////////////////////////////
	/// Static Helper Values
	/////////////////////////////////////////////////////////////////////////////
	// '#' is the stop char
	static const char DRIVER_STOP_CHAR{'\n'};
	// Wait up to a maximum of 3 seconds for serial input
	static constexpr const uint8_t DRIVER_TIMEOUT{3};
	// Maximum buffer for sending/receiving.
	static constexpr const uint8_t DRIVER_LEN{255};
};

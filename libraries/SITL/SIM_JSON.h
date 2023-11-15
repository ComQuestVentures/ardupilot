/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_JSON_ENABLED
#define HAL_SIM_JSON_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_JSON_ENABLED

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

class JSON : public Aircraft {
public:
    JSON(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new JSON(frame_str);
    }

    /*  Create and set in/out socket for JSON generic simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:

    struct servo_packet_16 {
        uint16_t magic = 18458; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };

    struct servo_packet_32 {
        uint16_t magic = 29569; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[32];
    };

    // default connection_info_.ip_address
    const char *target_ip = "127.0.0.1";

    // default connection_info_.sitl_ip_port
    uint16_t control_port = 9002;

    SocketAPM sock;

    uint32_t frame_counter;
    double last_timestamp_s;

    void output_servos(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);

    uint32_t parse_sensors(const char *json);

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;

    enum data_type {
        DATA_UINT64,
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR3F,
        DATA_VECTOR3D,
        QUATERNION,
        BOOLEAN,
    };

    struct {
        double timestamp_s;
        struct {
            Vector3f gyro;
            Vector3f accel_body;
        } imu;
        Vector3d position;
        Vector3f attitude;
        Quaternion quaternion;
        Vector3f velocity;
        Vector3f velocity_wind;
        float rng[6];
        float rc[12];
        float bat_volt;
        float bat_amp;
        struct {
            float direction;
            float speed;
        } wind_vane_apparent;
        float airspeed;
        bool no_time_sync;
    } state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;
        bool required;
    } keytable[30] = {
        { "", "timestamp", &state.timestamp_s, DATA_DOUBLE, true },
        { "imu", "gyro",    &state.imu.gyro, DATA_VECTOR3F, true },
        { "imu", "accel_body", &state.imu.accel_body, DATA_VECTOR3F, true },
        { "", "position", &state.position, DATA_VECTOR3D, true },
        { "", "attitude", &state.attitude, DATA_VECTOR3F, false },
        { "", "quaternion", &state.quaternion, QUATERNION, false },
        { "", "velocity", &state.velocity, DATA_VECTOR3F, true },
        { "", "rng_1", &state.rng[0], DATA_FLOAT, false },
        { "", "rng_2", &state.rng[1], DATA_FLOAT, false },
        { "", "rng_3", &state.rng[2], DATA_FLOAT, false },
        { "", "rng_4", &state.rng[3], DATA_FLOAT, false },
        { "", "rng_5", &state.rng[4], DATA_FLOAT, false },
        { "", "rng_6", &state.rng[5], DATA_FLOAT, false },
        {"","velocity_wind", &state.velocity_wind, DATA_VECTOR3F, false},
        {"windvane","direction", &state.wind_vane_apparent.direction, DATA_FLOAT, false},
        {"windvane","speed", &state.wind_vane_apparent.speed, DATA_FLOAT, false},
        {"", "airspeed", &state.airspeed, DATA_FLOAT, false},
        {"", "no_time_sync", &state.no_time_sync, BOOLEAN, false},
        { "rc", "rc_1", &state.rc[0], DATA_FLOAT, false },
        { "rc", "rc_2", &state.rc[1], DATA_FLOAT, false },
        { "rc", "rc_3", &state.rc[2], DATA_FLOAT, false },
        { "rc", "rc_4", &state.rc[3], DATA_FLOAT, false },
        { "rc", "rc_5", &state.rc[4], DATA_FLOAT, false },
        { "rc", "rc_6", &state.rc[5], DATA_FLOAT, false },
        { "rc", "rc_7", &state.rc[6], DATA_FLOAT, false },
        { "rc", "rc_8", &state.rc[7], DATA_FLOAT, false },
        { "rc", "rc_9", &state.rc[8], DATA_FLOAT, false },
        { "rc", "rc_10", &state.rc[9], DATA_FLOAT, false },
        { "battery", "voltage", &state.bat_volt, DATA_FLOAT, false },
        { "battery", "current", &state.bat_amp, DATA_FLOAT, false },
    };

    // Enum coresponding to the ordering of keys in the keytable.
    enum DataKey {
        TIMESTAMP   = 1U << 0,
        GYRO        = 1U << 1,
        ACCEL_BODY  = 1U << 2,
        POSITION    = 1U << 3,
        EULER_ATT   = 1U << 4,
        QUAT_ATT    = 1U << 5,
        VELOCITY    = 1U << 6,
        RNG_1       = 1U << 7,
        RNG_2       = 1U << 8,
        RNG_3       = 1U << 9,
        RNG_4       = 1U << 10,
        RNG_5       = 1U << 11,
        RNG_6       = 1U << 12,
        WIND_VEL    = 1U << 13,
        WIND_DIR    = 1U << 14,
        WIND_SPD    = 1U << 15,
        AIRSPEED    = 1U << 16,
        TIME_SYNC   = 1U << 17,
        RC_1        = 1U << 18,
        RC_2        = 1U << 19,
        RC_3        = 1U << 20,
        RC_4        = 1U << 21,
        RC_5        = 1U << 22,
        RC_6        = 1U << 23,
        RC_7        = 1U << 24,
        RC_8        = 1U << 25,
        RC_9        = 1U << 26,
        RC_10       = 1U << 27,
        BAT_VOLT    = 1U << 28,
        BAT_AMP     = 1U << 29,
    };
    uint32_t last_received_bitmask;
};

}

#endif  // HAL_SIM_JSON_ENABLED

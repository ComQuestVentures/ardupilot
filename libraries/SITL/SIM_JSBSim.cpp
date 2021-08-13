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
/*
  simulator connector for JSBSim
*/

#include "SIM_JSBSim.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 1

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(true),
    sock_fgfdm(true),
    initialised(false),
    jsbsim_script(nullptr),
    jsbsim_fgout(nullptr),
    created_templates(false),
    started_jsbsim(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL),
    last_timestamp_us(0)
{

    use_time_sync = false;
    home_is_set = false;

    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;

    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);

    printf("JSBSim backend started: control_port=%u fdm_port=%u\n",
           control_port, fdm_port);
}

/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;

    return true;
}


/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float pwms[16];

    for (uint16_t i=0; i<16; i++) {
        pwms[i] = (input.servos[i] - 1500)/500.0f;
    }

    asprintf(&buf,
             "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
             pwms[0], pwms[1], pwms[2], pwms[3],
             pwms[4], pwms[5], pwms[6], pwms[7],
             pwms[8], pwms[9], pwms[10], pwms[11],
             pwms[12], pwms[13], pwms[14], pwms[15]);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }

}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 0) != sizeof(fdm)) {}
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    float a_limit = GRAVITY_MSS*16;
    accel_body = Vector3f(constrain_float(fdm.A_X_pilot * FEET_TO_METERS, -a_limit, a_limit),
                          constrain_float(fdm.A_Y_pilot * FEET_TO_METERS, -a_limit, a_limit),
                          constrain_float(fdm.A_Z_pilot * FEET_TO_METERS, -a_limit, a_limit));

    float gyro_limit = 34.9f;
    gyro = Vector3f(constrain_float(fdm.phidot, -gyro_limit, gyro_limit),
                    constrain_float(fdm.thetadot, -gyro_limit, gyro_limit),
                    constrain_float(fdm.psidot, -gyro_limit, gyro_limit));

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;

    location.lat = fdm.latitude * 180.0 / 3.141592653589793; //NOTE: Maintain double precision
    location.lng = fdm.longitude * 180.0 / 3.141592653589793; //NOTE: Maintain double precision
    location.alt = fdm.agl*100 + home.alt;

    Quaternion quat(fdm.phi, fdm.theta, fdm.psi, fdm.alpha);
    quat.rotation_matrix(dcm);

    //NOTE: For some reason airspeed is not, so applying temporary hack here
    airspeed = norm(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS / 1.69f; //Airspeed in any direction (Note: ignores wind)
    airspeed_pitot = fdm.vcas * KNOTS_TO_METERS_PER_SECOND / 1.69f; //Airspeed in forward direction (Note: ignores wind)

    // update magnetic field
    update_mag_field_bf();

    // RC input from Typhon UDX
    rcin_chan_count = 8;
    rcin[0] = (fdm.right_aileron + 1.0f)*0.5f;
    rcin[1] = (-fdm.elevator + 1.0f)*0.5f;
    rcin[2] = (fdm.speedbrake + 1.0f)*0.5f;
    rcin[3] = (fdm.rudder + 1.0f)*0.5f;
    rcin[4] = (fdm.spoilers + 1.0f)*0.5f;
    rcin[7] = (fdm.spoilers + 1.0f)*0.5f;

    // Sync time
    uint64_t deltat_us = fdm.cur_time - last_timestamp_us;
    last_timestamp_us = fdm.cur_time;
    time_now_us += deltat_us;
    time_advance();

    //Set home
    if (!home_is_set) {
        set_start_location(location, sitl->opos.hdg.get());
        home_is_set = true;
    }

}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}

void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    drain_control_socket();
}

}
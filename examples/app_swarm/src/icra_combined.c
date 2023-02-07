/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>


#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "estimator_kalman.h"

#define SEQUENCE_SPEED 1.0f
double alpha = 3.0;
double beta = 0.0;
double gama = 1.0;
double kappa = 10.0;
double sigma_const = 0.6;
double sigma = 0.5;
double var_a = 81;
double var_b = 50;
double angle = 0.0;
double r = 0.0;
double d = 0.0;
double grad = 0.0;
double grad_x = 0.0;
double grad_y = 0.0;
double grad_const_x = 25.0769;
double grad_const_y = 25.00;
double krep = 50.0;
double L0 = 0.5;
double umax = 0.15;
double wmax = 1.5708;
double K1 = 0.2;
double K2 = 0.1;
double epsilon = 12.0;
double neg_xs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ys[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double neg_hs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double self_pos[] = {0.0, 0.0, 0.0};
bool neg_alive[] = {false, false, false, false, false, false, false};
bool form_g_upd = false;
double px = 0.0;
double temp_px = 0.0;
double temp_py = 0.0;
double py = 0.0;
double hx = 0.0;
double hy = 0.0;
double sum_cosh = 0.0;
double sum_sinh = 0.0;
double rx = 0.0;
double ry = 0.0;
double u = 0.0;
double u_add = 0.05;
double w = 0.0;
double fx = 0.0;
double fy = 0.0;
double gx = 0.0;
double gy = 0.0;
double fx_raw = 0.0;
double fy_raw = 0.0;
double vx = 0.0;
double vy = 0.0;
double distance = 0.0;
double ij_ang = 0.0;
// double c_form_angles[] = {0.0, 0.0, 1.25664, 2.51326, 3.76991, 5.02655};
// double c_form_angles_r[] ={0.0, 5.02655, 3.76991, 2.51326, 1.25664, 0.0};
double c_form_angles[] = {0.0, 0.0, 1.0472, 2.0944, 3.1416, 4.1888, 5.236};
double c_form_angles_r[] ={0.0, 5.236, 4.1888, 3.1416, 2.0944, 1.0472, 0.0};
double obs_p_x[] = {0.0, 1.5, 3.25, 4.5, 4.5, 3.25, 1.5};
double obs_p_y[] = {0.0, 1.0, 1.0, 1.0, 2.5, 2.5, 2.5};
uint8_t scan_p = 0;
double dist_to_scanp = 0.0;
static float trajectoryDurationMs = 0.0f;

static struct poly4d sequence[] = {
  {.duration = 1.07565, .p = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}, {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}, {0.05,0.0,0.0,0.0,1.29979,-2.08392,1.25117,-0.271964}, {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}}},
  {.duration = 1.12385, .p = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}, {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}, {0.275,0.38825,-0.0000000000000198003,-0.103889,-0.164115,-0.08355442,0.313605,-0.12548}, {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,}}}
};
static float sequenceTime(struct poly4d sequence[], int count) {
  float totalDuration = 0.0f;

  for (int i = 0; i < count; i++) {
    totalDuration += sequence[i].duration;
  }

  return totalDuration;
}
static void defineTrajectory() {
  const uint32_t polyCount = sizeof(sequence) / sizeof(struct poly4d);
  trajectoryDurationMs = 1000 * sequenceTime(sequence, polyCount);
  crtpCommanderHighLevelWriteTrajectory(0, sizeof(sequence), (uint8_t*)sequence);
  crtpCommanderHighLevelDefineTrajectory(1, CRTP_CHL_TRAJECTORY_TYPE_POLY4D, 0, polyCount);
}

double total_flight = 0.0;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdiftakeoff;
static paramVarId_t paramIdifterminate;
static paramVarId_t paramIdifheading;
static paramVarId_t paramIdfmode;
static paramVarId_t paramIdgoalx;
static paramVarId_t paramIdgoaly;
static paramVarId_t paramIdformation;
static void resetKalman() { paramSetInt(paramIdResetKalman, 1); }
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

typedef struct {
  uint8_t id;
  float x;
  float y;
  float h;
  } _coords;

typedef enum {
    idle,
    takingOff,
    onAir,
    land,
} State;

static State state = idle;


void p2pcallbackHandler(P2PPacket *p)
{
  //static TickType_t last_check=0;
  //TickType_t current_check = xTaskGetTickCount();

  _coords other_coords;
  memcpy(&other_coords, p->data, sizeof(other_coords));
  uint8_t other_id = other_coords.id;
  //for (int i = 0; i < 5; i++) 
  //  DEBUG_PRINT("B i: %i, Other ID: %i, neg_alive[i]: %s \n", i, other_id, neg_alive[i] ? "true" : "false"); 
  neg_alive[other_id - 1] = true;
  //for (int i = 0; i < 5; i++) 
  //  DEBUG_PRINT("A i: %i, Other ID: %i, neg_alive[i]: %s \n", i, other_id, neg_alive[i] ? "true" : "false"); 
  float other_X = other_coords.x;
  float other_Y = other_coords.y;
  float other_H = other_coords.h;
  neg_xs[other_id - 1] = (double)other_X;
  //DEBUG_PRINT("neg_xs:%f, neg_ys:%f - %lu\n", neg_xs[other_id-1], neg_ys[other_id-1], current_check - last_check );  
  neg_ys[other_id - 1] = (double)other_Y;
  neg_hs[other_id - 1] = (double)other_H;
  //last_check = current_check;
}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
}

void appMain()
{
  // DEBUG_PRINT("Waiting for activation ...\n");

    static setpoint_t setpoint;
    static float heading_log;
    heading_log = 99;
    static uint8_t iftakeoff;
    static uint8_t ifterminate;
    static uint8_t ifheading;
    static uint8_t _fmode;
    static uint8_t _formation;
    static float _goal_x;
    static float _goal_y;
    _fmode = 1;
    _goal_x = 3.0;
    _goal_y = 2.25;
    iftakeoff = 0;
    ifterminate = 0;
    ifheading = 1;

    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");

    static P2PPacket p_reply;
    p_reply.port=0x00;
    
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p2pRegisterCB(p2pcallbackHandler);
    _coords self_coords;
    self_coords.id = my_id;

    // float myLogValueFunction(uint32_t timestamp, void* data) {
    //     return &heading_log;
    // }

    // logByFunction_t myLogger = {.aquireFloat = myLogValueFunction, .data = 0};
    LOG_GROUP_START(headingLog)
    LOG_ADD_CORE(LOG_FLOAT, heading, &heading_log)
    LOG_GROUP_STOP(myGroup)

    PARAM_GROUP_START(fmodes)
    PARAM_ADD_CORE(PARAM_UINT8, if_takeoff, &iftakeoff)
    PARAM_ADD_CORE(PARAM_UINT8, if_terminate, &ifterminate)
    PARAM_ADD_CORE(PARAM_UINT8, if_heading, &ifheading)
    PARAM_ADD_CORE(PARAM_UINT8, fmode, &_fmode)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_x, &_goal_x)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_y, &_goal_y)
    PARAM_ADD_CORE(PARAM_UINT8, formation, &_formation)
    PARAM_GROUP_STOP(fmodes)

    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdResetKalman = paramGetVarId("kalman", "resetEstimation");
    paramIdiftakeoff = paramGetVarId("fmodes", "if_takeoff");
    paramIdifterminate = paramGetVarId("fmodes", "if_terminate");
    paramIdfmode = paramGetVarId("fmodes", "fmode");
    paramIdifheading = paramGetVarId("fmodes", "if_heading");
    paramIdgoalx = paramGetVarId("fmodes", "goal_x");
    paramIdgoaly = paramGetVarId("fmodes", "goal_y");
    paramIdformation = paramGetVarId("fmodes", "formation");
    resetKalman();
    enableHighlevelCommander();
    defineTrajectory();
    srand((unsigned int)xTaskGetTickCount());
    double test_rand = (double)rand()/(double)(RAND_MAX/6.28);
    DEBUG_PRINT("random is %f\n", test_rand);
    vTaskDelay(M2T(8000));


  // TickType_t time_begin=0, time_end=0;
  // const TickType_t LOOP_DURATION_TARGET = 50;
  while(1) {
    // time_begin = xTaskGetTickCount();
    // DEBUG_PRINT("iftakeoff%d\n", iftakeoff);
    if (state == idle) {

      if (iftakeoff == 0) {
        iftakeoff = paramGetInt(paramIdiftakeoff);
        vTaskDelay(M2T(100));
      }

      if (iftakeoff == 1) {
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelTakeoff(0.5, 2.5);
        // crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, true, false);
        vTaskDelay(M2T(9000));
        // vTaskDelay(M2T(5000));
        state = takingOff;
        self_pos[2] = (double)rand()/(double)(RAND_MAX/6.28);
        // self_pos[2] = 0.0;
      }
    }

    if (state == takingOff) {
      if (crtpCommanderHighLevelIsTrajectoryFinished()) 
      {state = onAir;}
      // state = onAir;
    }

    if (state == onAir) {
      px = 0.0;
      py = 0.0;
      hx = 0.0;
      hy = 0.0;
      rx = 0.0;
      ry = 0.0;
      self_coords.x = logGetFloat(idX);
      self_coords.y = logGetFloat(idY);
      self_coords.h = (float)self_pos[2];
      heading_log = self_coords.h;
      self_pos[0] = (double)self_coords.x;
      self_pos[1] = (double)self_coords.y;

      if (_fmode == 1) {
        scan_p = 0;
        form_g_upd = false;
        sigma = sigma_const;
        kappa = 0.0;
        u_add = 0.05;

        if (ifheading == 1) {
          K1 = 0.1;
          K2 = 0.6;
          alpha = 1.0;
          beta = 2.0;
      }
        else if (ifheading == 0) {
          K1 = 0.3;
          K2 = 0.1;
          alpha = 1.0;
          beta = 0.0;
        }
      }
      else if (_fmode == 2) {
        form_g_upd = false;
        scan_p = 0;
        grad_x = ceil(self_pos[0] * grad_const_x);
        grad_y = ceil(self_pos[1] * grad_const_y);
        if (grad_x >= 163) {grad_x = 162.0;}
        if (grad_x <= 0) {grad_x = 0.0;}
        if (grad_y >= 100) {grad_y = 99.0;}
        if (grad_y <= 0) {grad_y = 0;}
        angle = atan2(grad_y-50, grad_x-81);
        r = (var_a * var_b) / (sqrt(var_a*var_a*sin(angle)*sin(angle) + var_b*var_b*cos(angle)*cos(angle)));
        d = sqrt((grad_y - 50)*(grad_y - 50) + (grad_x-81)*(grad_x-81));
        grad = 255.0 - (d/r)*255.0;
        if (grad >= 255.0) {grad = 255.0;}
        if (grad <= 0.0) {grad = 0.0;}
        sigma = 0.5 + (grad/255.0)*0.3;
        kappa = 0.0;
        u_add = 0.05;

        if (ifheading == 1) {
        K1 = 0.1;
        K2 = 0.6;
        alpha = 1.0;
        beta = 2.0;
    }
      else if (ifheading == 0) {
        K1 = 0.2;
        K2 = 0.1;
        alpha = 3.0;
        beta = 0.0;
      }
      }
      else if (_fmode == 3) {
        scan_p = 0;
        form_g_upd = false;
        sigma = sigma_const;
        _goal_x = paramGetFloat(paramIdgoalx);
        _goal_y = paramGetFloat(paramIdgoaly);
        gx = (double)_goal_x - self_pos[0];
        gy = (double)_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        kappa = 10.0;
        u_add = 0.05;


        if (ifheading == 1) {
        K1 = 0.1;
        K2 = 0.6;
        alpha = 1.0;
        beta = 2.0;
    }
      else if (ifheading == 0) {
        K1 = 0.3;
        K2 = 0.1;
        alpha = 1.0;
        beta = 0.0;
      }        
      }

      else if (_fmode == 4) {
        scan_p = 0;
        sigma = sigma_const;
        K1 = 0.1;
        K2 = 0.6;
        alpha = 1.0;
        beta = 0.0;  
        _goal_x = paramGetFloat(paramIdgoalx);   
        _goal_y = paramGetFloat(paramIdgoaly);
        double temp_goal_x = (double)_goal_x + 1*cos(c_form_angles[my_id-1]);
        double temp_goal_y = (double)_goal_y + 1*sin(c_form_angles[my_id-1]);
        gx = (double)temp_goal_x - self_pos[0];
        gy = (double)temp_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        u_add = 0.00;
        kappa = 10;

      }

      else if (_fmode == 5) {
        scan_p = 0;
        sigma = sigma_const;
        K1 = 0.1;
        K2 = 0.6;
        alpha = 1.0;
        beta = 0.0;  
        _goal_x = paramGetFloat(paramIdgoalx);   
        _goal_y = paramGetFloat(paramIdgoaly);
        double temp_goal_x = (double)_goal_x + 1*cos(c_form_angles_r[my_id-1]);
        double temp_goal_y = (double)_goal_y + 1*sin(c_form_angles_r[my_id-1]);
        gx = (double)temp_goal_x - self_pos[0];
        gy = (double)temp_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        u_add = 0.00;
        kappa = 10;

      }     

      else if (_fmode == 6) {
        sigma = sigma_const;
        K1 = 0.1;
        K2 = 0.6;
        alpha = 1.0;
        beta = 0.0;  

        double temp_goal_x = _goal_x;
        double temp_goal_y = _goal_y;

        if (scan_p == 0) {
          temp_goal_x = obs_p_x[my_id - 1];
          temp_goal_y = obs_p_y[my_id - 1];
          dist_to_scanp = sqrt( ((self_pos[0] - temp_goal_x)*(self_pos[0] - temp_goal_x)) + ((self_pos[1] - temp_goal_y)*(self_pos[1] - temp_goal_y)) );
          if (dist_to_scanp < 0.1) {scan_p = 1;}
        }
        else if (scan_p == 1) {
          temp_goal_x = obs_p_x[my_id - 1] + 0.5;
          temp_goal_y = obs_p_y[my_id - 1]; 
          dist_to_scanp = sqrt( ((self_pos[0] - temp_goal_x)*(self_pos[0] - temp_goal_x)) + ((self_pos[1] - temp_goal_y)*(self_pos[1] - temp_goal_y)) );
          if (dist_to_scanp < 0.1) {scan_p = 2;}         
        }
        else if (scan_p == 2) {
          temp_goal_x = obs_p_x[my_id - 1] + 0.5;
          temp_goal_y = obs_p_y[my_id - 1] + 0.5;    
          dist_to_scanp = sqrt( ((self_pos[0] - temp_goal_x)*(self_pos[0] - temp_goal_x)) + ((self_pos[1] - temp_goal_y)*(self_pos[1] - temp_goal_y)) );
          if (dist_to_scanp < 0.1) {scan_p = 3;}     
        }   
        else if (scan_p == 3) {
          temp_goal_x = obs_p_x[my_id - 1];
          temp_goal_y = obs_p_y[my_id - 1] + 0.5;     
          dist_to_scanp = sqrt( ((self_pos[0] - temp_goal_x)*(self_pos[0] - temp_goal_x)) + ((self_pos[1] - temp_goal_y)*(self_pos[1] - temp_goal_y)) );
          if (dist_to_scanp < 0.1) {scan_p = 0;}    
        }      

        // DEBUG_PRINT("scan_p: %i, temp_goal_x: %f, temp_goal_y: %f \n", scan_p, temp_goal_x, temp_goal_y);
        gx = (double)temp_goal_x - self_pos[0];
        gy = (double)temp_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        u_add = 0.05;
        kappa = 10;

      }       

      sum_cosh = cos(self_pos[2]);
      sum_sinh = sin(self_pos[2]);

      // DEBUG_PRINT("fmode: %i, ifheading: %i, goalX: %f, goalY: %f \n", _fmode, ifheading, (double)_goal_x, (double)_goal_y);
      // DEBUG_PRINT("K1: %f, K2: %f, alpha: %f, beta: %f, kappa: %f \n", K1, K2, alpha, beta, kappa);

      for (int i = 0; i < 7; i++) 
      {
        // DEBUG_PRINT("i: %i, My ID: %i, neg_alive[i]: %s \n", i, my_id, neg_alive[i] ? "true" : "false"); 
        if ( neg_alive[i] && (neg_xs[i] > 0.0) && ((i+1) != my_id) )
        {
          // DEBUG_PRINT("i: %d\n", i);
          distance = sqrt( ((self_pos[0] - neg_xs[i])*(self_pos[0] - neg_xs[i])) + ((self_pos[1] - neg_ys[i])*(self_pos[1] - neg_ys[i])) );
          // DEBUG_PRINT("burada\n");
          ij_ang = atan2((neg_ys[i] - self_pos[1]), (neg_xs[i] - self_pos[0]));
          // DEBUG_PRINT("Bearing is: %f\n", ij_ang);
          temp_px = ((-epsilon * (( 2.*( pow(sigma, 4.) / pow(distance, 5.) ) ) - ( pow(sigma, 2.) / pow(distance, 3.) ) ) ) * cos(ij_ang) );
          temp_py = ((-epsilon * (( 2.*( pow(sigma, 4.) / pow(distance, 5.) ) ) - ( pow(sigma, 2.) / pow(distance, 3.) ) ) ) * sin(ij_ang) );

          if (_fmode == 4) {
            if (distance > (sigma * sqrt(2)*0.7)) {
              temp_px = 0.0;
              temp_py = 0.0;
            }
          }

          px = px + temp_px;
          py = py + temp_py;

          sum_cosh += cos(neg_hs[i]);
          sum_sinh += sin(neg_hs[i]);
          // DEBUG_PRINT("In for this case! \n"); 
        }
      }

      hx = sum_cosh / (sqrt((sum_cosh*sum_cosh) + (sum_sinh*sum_sinh))); 
      hy = sum_sinh / (sqrt((sum_cosh*sum_cosh) + (sum_sinh*sum_sinh))); 

      if (self_pos[0]<0.5) 
      {
        rx += krep * ( (1./self_pos[0]) - (1./L0) ) * ( cos(0.0)/pow(self_pos[0],3.) );
        ry += krep * ( (1./self_pos[0]) - (1./L0) ) * ( sin(0.0)/pow(self_pos[0],3.) );
      }
      if (self_pos[1]>3.5) 
      {
        rx += krep * ( (1./(4.0-self_pos[1])) - (1./L0) ) * ( cos(-1.5708)/pow((4.0-self_pos[1]),3.) );
        ry += krep * ( (1./(4.0-self_pos[1])) - (1./L0) ) * ( sin(-1.5708)/pow((4.0-self_pos[1]),3.) );
      }
      if (self_pos[0]>6.0) 
      {
        rx += krep * ( (1./(6.5-self_pos[0])) - (1./L0) ) * ( cos(3.14)/pow((6.5-self_pos[0]),3.) );
        ry += krep * ( (1./(6.5-self_pos[0])) - (1./L0) ) * ( sin(3.14)/pow((6.5-self_pos[0]),3.) );
      }
      if (self_pos[1]<0.5) 
      {
        rx += krep * ( (1./(self_pos[1])) - (1./L0) ) * ( cos(1.5708)/pow((self_pos[1]),3.) );
        ry += krep * ( (1./(self_pos[1])) - (1./L0) ) * ( sin(1.5708)/pow((self_pos[1]),3.) );
      }

      fx_raw = alpha * px + beta * hx + gama * rx + kappa * gx;
      fy_raw = alpha * py + beta * hy + gama * ry + kappa * gy;

      fx = sqrt(fx_raw*fx_raw + fy_raw*fy_raw) * cos(atan2(fy_raw, fx_raw) - self_pos[2]);
      fy = sqrt(fx_raw*fx_raw + fy_raw*fy_raw) * sin(atan2(fy_raw, fx_raw) - self_pos[2]);



      u = K1 * fx + u_add;
      // DEBUG_PRINT("U is: %f\n", u);
      w = K2 * fy;

      if (u>umax) {u = umax;}
      else if (u<0) {u = 0.0;}

      if (w>wmax) {w = wmax;}
      else if (w<-wmax) {w = -wmax;}  
      
      vx = (float)(u * cos(self_pos[2]));
      vy = (float)(u * sin(self_pos[2]));
      self_pos[2] = self_pos[2] + w*0.05; // 0.05s should be the loop time
      self_pos[2] = atan2(sin(self_pos[2]), cos(self_pos[2])); 
      
      setHoverSetpoint(&setpoint, vx, vy, 0.5, 0.0);
      commanderSetSetpoint(&setpoint, 3);  

      memcpy(p_reply.data, &self_coords, sizeof(self_coords));
      p_reply.size = sizeof(self_coords)+1;
      radiolinkSendP2PPacketBroadcast(&p_reply);
      ifterminate = paramGetInt(paramIdifterminate);
      _fmode = paramGetInt(paramIdfmode);
      ifheading = paramGetInt(paramIdifheading);
      _formation = paramGetInt(paramIdformation);
      // DEBUG_PRINT("fmode: %i \n", _fmode);
      vTaskDelay(M2T(50));
      // time_end = xTaskGetTickCount();
      // TickType_t loop_duration = time_end - time_begin;
      // if (loop_duration < LOOP_DURATION_TARGET ) {
        // vTaskDelay(M2T(LOOP_DURATION_TARGET - loop_duration));
      // } else {
        // DEBUG_PRINT("WARNING! loop took %lu ms, which is more than the target %lu ms\n", loop_duration, LOOP_DURATION_TARGET);
      // }
      total_flight += 50;     
      if (total_flight > 240000 || ifterminate == 1) {
        state = land;
      }
    }

    if(state == land) {
      crtpCommanderHighLevelLand(0.03, 1.0);
      vTaskDelay(M2T(2000));
      return;
    }
  }
}


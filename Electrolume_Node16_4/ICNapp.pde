 #ifdef RUNTIME
// *************************************************************************************
//  ICN APP CONFIG
// *************************************************************************************

#define icn_my_talkPin 12
#define icn_my_address 16                       // my unique node ID
#define icn_my_appName "Electrolume"              // app name: 32 chars max
#define icn_my_appAuthor "Mads Høbye"             // app author: 32 chars max
#define icn_my_appVersion 101

#define icn_my_params_in_num 5                    // number of control parameters accepted from host
#define icn_my_params_in_type float14            // type of control parameters accepted from host
                                                  // valid options are:
                                                  // int8, int14, float8, float14, sfloat8, sfloat14

#define icn_my_params_out_num 23                  // number of data items returned to host
#define icn_my_params_out_type int8              // type of data items returned to host
                                                  // valid options are:
                                                  // int8, int14, float8, float14, sfloat8, sfloat14

#define icn_my_signals_num 0                      // number of signals supported

// *************************************************************************************

// #define icn_mega128 // UNCOMMENT IF WIRING BOARD USED

// WIRING BOARD OPTION ONLY WORKS WITH MODIFIED WIRING VERSION AVAILABLE FROM ILLUTRON
// The PDE preprocessor in the hacked version omits inclusion of hardwareserial.h

// *************************************************************************************
//  ICN APP HOOKS 
// *************************************************************************************

int icn_app_start(int sender, int socket, boolean broadcast)
{
  // call from network with standard signal "APP START"
  // if broadcast is high then signal is for everybody, not just me
  return 0;
}

int icn_app_stop(int sender, int socket, boolean broadcast)
{
  // call from network with standard signal "APP STOP"
  // if broadcast is high then signal is for everybody, not just me
  return 0;
}

int icn_app_reset(int sender, int socket, boolean broadcast)
{
  // call from network with standard signal "APP RESET"
  // if broadcast is high then signal is for everybody, not just me
  return 0;
}

int icn_app_user(int sender, int socket, int length, unsigned char *usermessage, boolean broadcast)
{
  // user message of specified length available in usermessage buffer.
  return 0;
}

int icn_app_signal(int sender, int socket, int signal, boolean broadcast)
{
  return 0;
}
/* Just for reference
float maxSlider =3015;
float cutoffSlider =     100;
float factorLightSlider = 1.3f
float changeSLider = 1.3f;
float energySlider = 0.3;
*/

int icn_app_param_changed(int sender, int socket, int index, boolean broadcast)
{
  // control parameter was updated remotely!
  
 switch(index)
 {
    case 0:
       maxSlider = readRemoteFloat(index) * 3015.0f *2.0f;
       break;
    case 1:
       cutoffSlider = readRemoteFloat(index) * 100.0f * 2.0f;
       break;
    case 2:
       factorLightSlider = readRemoteFloat(index) * 2.5f * 2.0f;
       break;
    case 3:
       changeSLider = readRemoteFloat(index) * 2.5f * 2.0f;
       break;
    case 4:
       energySlider = readRemoteFloat(index) * 0.5f * 2.0f;
       break;
 }

  return 0;
}

void icn_app_dumped(int sender, int socket)
{
  // return buffer was dumped
}

void icn_app_spoolStopped(int spoolPort, boolean myBumper)
{
  // spool playback complete event
}
 #endif

#ifndef H_JOYSTICK
#define H_JOYSTICK

#ifdef __WIN32__
#include <windows.h>
#include <mmsystem.h>
#define JOYSTICK_DEVICE         ""
#endif

typedef struct
{
#ifdef __WIN32__
  JOYINFOEX joy;
#endif
	int device;
	int event;
	float axis[10];
	int axis_i[10];
	char buttons[10];
} joystick_status;

#ifdef __cplusplus
extern "C"
{
#endif

int joystick_init(int dev, joystick_status* js);

int joystick(joystick_status* js);

void joystick_done(joystick_status* js);

#ifdef __cplusplus
}
#endif

#endif

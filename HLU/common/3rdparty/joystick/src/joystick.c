#ifdef __WIN32__
#include <math.h>
#define JOYSTICK_DEVICE		""
#else
#include <linux/joystick.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#define JOYSTICK_DEVICE		"/dev/input/js%d"
#endif
#include <joystick.h>

int joystick_init(int dev, joystick_status* js)
{
#ifdef __WIN32__
	js->device = dev;
	js->joy.dwSize = sizeof(JOYINFOEX);
	js->joy.dwFlags = JOY_RETURNALL;
	return joyGetPosEx((unsigned int) js->device, &js->joy) == JOYERR_NOERROR;
#else
	int j;
	char device[256];

	snprintf(device, 256, JOYSTICK_DEVICE, dev);
	j = open(device, O_RDONLY | O_NONBLOCK);
	if (j == -1)
		return 0;
	js->device = j;
	return 1;
#endif
}

int joystick(joystick_status* js)
{
#ifdef __WIN32__
	int i;
	if (joyGetPosEx((unsigned int) js->device, &js->joy) != JOYERR_NOERROR)
	  return 0;
	js->axis_i[0] = js->joy.dwXpos;
	js->axis_i[1] = js->joy.dwYpos;
	js->axis_i[2] = js->joy.dwZpos;
	js->axis_i[3] = js->joy.dwRpos;
	js->axis_i[4] = js->joy.dwUpos;
	js->axis_i[5] = js->joy.dwVpos;
	for (i = 0; i < 6; i++)
	  js->axis[i] = (js->axis_i[i] - 32767) / (float) 32767;
	for (i = 0; i < 10; i++)
	  js->buttons[i] = (js->joy.dwButtons & ((int) pow(2, i))) != 0;
	return 1;
#else
	struct js_event event[32];
	int i;
	int len;
	while ((len = read(js->device, &event, sizeof(event))) > 0)
	{
		len /= sizeof(event[0]);
		for (i = 0; i < len; i++)
		{
			switch (event[i].type & ~JS_EVENT_INIT)
			{
			case JS_EVENT_BUTTON:
				js->buttons[event[i].number] = event[i].value;
				js->event = 1;
				break;
			case JS_EVENT_AXIS:
				js->axis[event[i].number] = (float) event[i].value / SHRT_MAX;
				js->axis_i[event[i].number] = event[i].value;
				js->event = 1;
				break;
			default:
				js->event = 0;
				break;
			}
		}
	}
	return 1;
#endif
}

void joystick_done(joystick_status* js)
{
#ifdef __WIN32__
#else
	close(js->device);
#endif
}


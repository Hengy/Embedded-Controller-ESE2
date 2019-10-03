/*
 ============================================================================
 Name        : RobotController.c
 Author      : Matt Hengeveld; Kevin MacIntosh
 Version     :
 Copyright   : None
 Description : ESE Robot Controller
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#define REPEAT_PERIOD 	250000
#define JSDEADZONE		5000

#define TANKCONTROLS		// comment out to use differential steering

int read_event(int fd, struct js_event *event) {
	ssize_t bytes;

	bytes = read(fd, event, sizeof(*event));

	if (bytes == sizeof(*event))
		return 0;

	return -1;
}

size_t get_axis_count(int fd) {
	__u8 axes;

	if (ioctl(fd, JSIOCGAXES, &axes) == -1)
		return 0;

	return axes;
}

size_t get_button_count(int fd) {
	__u8 buttons;
	if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
		return 0;

	return buttons;
}

struct axis_state {
	short x, y;
};

size_t get_axis_state(struct js_event *event, struct axis_state axes[3]) {

	size_t axis = event->number/2;

	if (axis < 3) {
		if (event->number % 2 == 0)
			axes[axis].x = event->value;
		else
			axes[axis].y = event->value;
	}

	return axis;
}

int RS232_init (void) {

	struct termios config;

	int baudrate = B115200;

	int serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd == -1) {
		printf("Serial Error %i, %s\n", errno, strerror(errno));
		return -1;
	} else {
		printf("Serial Initialized\n");
	}

	memset (&config, 0, sizeof(config));
	cfsetispeed (&config, baudrate);
	cfsetospeed (&config, baudrate);
	config.c_cflag |= CS8 | CLOCAL | CREAD;
	config.c_iflag = IGNPAR | ICRNL;
	config.c_oflag = 0;
	config.c_lflag = 0;
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 1;

	tcflush (serial_fd, TCIFLUSH);
	tcsetattr (serial_fd, TCSANOW, &config);

	return serial_fd;
}

void send_robot_cmd(int serial_fd, char *cmd) {
	write(serial_fd, cmd, 3);
}

void send_robot_datacmd(int serial_fd, char *cmd, char *data) {
	char send_buf[64];
	memcpy(send_buf, data, 61);
	memcpy(send_buf+60, cmd, 3);
	write(serial_fd, send_buf, 64);
}

int main(void) {
	puts("Initialting Robot Control"); /* prints Initialting Robot Control */

	struct js_event event;
	__u32 prev_time;
	struct axis_state axes[3] = {0};
	size_t axis;
	char data_buf[61];
	char cmd_buf[3];

	double x, y, result_angle;

	clock_t prev_cmd_time, now_time;
	prev_cmd_time = clock()/1000;

	int camera_repeat = 0;
	int camera_repeat_sig = 0;

	int joystick_fd = open("/dev/input/js2", O_RDONLY );

	fcntl(joystick_fd, F_SETFL, O_NONBLOCK);

	if (joystick_fd == -1) {
		perror("Could not open joystick");
	}

	int serial_fd = RS232_init();

	while (1) {
		read_event(joystick_fd, &event);
		if (prev_time != event.time) {
			switch (event.type)
			{
			case JS_EVENT_BUTTON:
				printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
				if (event.value == 0) {
					//printf("Button %u\n", event.number);
					switch (event.number) {
					case 0:	// X; robot stop
						send_robot_cmd(serial_fd, "SRX");
						break;

					case 1:	// A

						break;

					case 2:	// B; camera straight
						send_robot_cmd(serial_fd, "SCX");
						break;

					case 3:	// Y; home
						send_robot_cmd(serial_fd, "MHX");
						break;

					case 4:	// LB; 90 degree left turn
						send_robot_cmd(serial_fd, "LHX");
						break;

					case 5:	// RB; 90 degree right turn
						send_robot_cmd(serial_fd, "RHX");
						break;

					case 10:
					case 11:
						memset(data_buf, 0x00, 61);
						send_robot_datacmd(serial_fd, "MRX", data_buf);
						break;

					default:
						//printf("Unmapped button %u\n", event.number);
						break;
					}
				}
				break;

			case JS_EVENT_AXIS:
				axis = get_axis_state(&event, axes);
				if (axis < 3)
					printf("Axis %u at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
					switch (axis) {
					case 2:	// dpad
						if (axes[2].y > 32000) {	// dpad down
							send_robot_cmd(serial_fd, "DCX");
							camera_repeat_sig = 1;
							camera_repeat = REPEAT_PERIOD;
						} else if (axes[2].y < -32000) {	// dpad up
							send_robot_cmd(serial_fd, "UCX");
							camera_repeat_sig = 2;
							camera_repeat = REPEAT_PERIOD;
						} else if (axes[2].x > 32000) {	// dpad right
							send_robot_cmd(serial_fd, "RCX");
							camera_repeat_sig = 3;
							camera_repeat = REPEAT_PERIOD;
						} else if(axes[2].x < -32000) {	// dpad left
							send_robot_cmd(serial_fd, "LCX");
							camera_repeat_sig = 4;
							camera_repeat = REPEAT_PERIOD;
						}
						if ((axes[2].x == 0) && (axes[2].y == 0)) {
							camera_repeat_sig = 0;
							printf("repeat stopped\n");
						}
						break;

					case 1:
					case 0:
#ifndef TANKCONROLS	// use tank controls

						memset(data_buf, 0x00, 61);
						if (axes[1].y > JSDEADZONE) {		// right joystick reverse
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 0;
						} else if (axes[1].y < -JSDEADZONE) {	// forward
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 'F';
						}

						if (axes[0].y > JSDEADZONE) {		// left joystick reverse
							data_buf[59] = (abs(axes[0].y*100) / 32767);
							data_buf[58] = 0;
						} else if (axes[0].y < -JSDEADZONE) {	// forward
							data_buf[59] = (abs(axes[0].y*100) / 32767);
							data_buf[58] = 'F';
						}

						send_robot_datacmd(serial_fd, "MRX", data_buf);
#else	// use differential steering
						memset(data_buf, 0x00, 61);

						x = (axes[1].x + 1) / 64;	// result is 1 - 512
						y = ((axes[1].y*-1) + 1) / 64;	// result is 1 - 512
						result_angle = ((atan2(x, y) * 180) / 3.14159)+180;
						//printf("Angle: %f", result_angle);

						if ((result_angle >= 45) && (result_angle < 135)) {
							// L: reverse
							data_buf[59] = (abs(axes[1].y*100) / 32767);
							data_buf[58] = 0;
							// R: forward
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 'F';
						} else if ((result_angle >= 135) && (result_angle < 225)) {
							// L: forward
							data_buf[59] = (abs(axes[1].y*100) / 32767);
							data_buf[58] = 'F';
							// R: forward
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 'F';
						} else if ((result_angle >= 225) && (result_angle < 315)) {
							// L: forward
							data_buf[59] = (abs(axes[1].y*100) / 32767);
							data_buf[58] = 'F';
							// R: reverse
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 0;
						} else {
							// L: reverse
							data_buf[59] = (abs(axes[1].y*100) / 32767);
							data_buf[58] = 0;
							// R: reverse
							data_buf[57] = (abs(axes[1].y*100) / 32767);
							data_buf[56] = 0;
						}
						send_robot_datacmd(serial_fd, "MRX", data_buf);
#endif
						break;

					}
				break;

			default:
				break;
			}



			prev_time = event.time;
		}

		now_time = clock()/1000;

		if ((now_time - prev_cmd_time) > 100) {
			prev_cmd_time = now_time;
			switch (camera_repeat_sig) {
			case 1:
				send_robot_cmd(serial_fd, "DCX");
				printf("repeat\n");
				break;

			case 2:
				send_robot_cmd(serial_fd, "UCX");
				printf("repeat\n");
				break;

			case 3:
				send_robot_cmd(serial_fd, "RCX");
				printf("repeat\n");
				break;

			case 4:
				send_robot_cmd(serial_fd, "LCX");
				printf("repeat\n");
				break;

			default:

				break;

			}
		}
	}
	pclose(joystick_fd);

	return EXIT_SUCCESS;
}



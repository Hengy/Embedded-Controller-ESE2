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

#define REPEAT_PERIOD 250000

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
	write(serial_fd, cmd, 3);
	write(serial_fd, data, 61);
}

int main(void) {
	puts("Initialting Robot Control"); /* prints Initialting Robot Control */

	struct js_event event;
	__u32 prev_time;
	struct axis_state axes[3] = {0};
	size_t axis;
	char data_buf[61];
	char cmd_buf[3];

	clock_t prev_cmd_time, now_time;
	prev_cmd_time = clock()/1000;

	printf("Time: %d\n", prev_cmd_time);

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
				//printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
				if (event.value == 0) {
					printf("Button %u\n", event.number);
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

					default:
						printf("Unmapped button %u\n", event.number);
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
						if (axes[1].y > 25000) {
							memset(data_buf, 0x00, 61);
							data_buf[59] = 100;
							data_buf[57] = 100;
							send_robot_datacmd(serial_fd, "MRX", data_buf);
						} else if (axes[1].y < -25000) {
							memset(data_buf, 0x00, 61);
							data_buf[59] = 100;
							data_buf[58] = 'F';
							data_buf[57] = 100;
							data_buf[56] = 'F';
							send_robot_datacmd(serial_fd, "MRX", data_buf);
						}
						if ( ((axes[1].x < 8000) && (axes[1].x > -8000)) && ((axes[1].y < 8000) && (axes[1].y > -8000)) ) { //dead zone
							memset(data_buf, 0x00, 61);
							send_robot_datacmd(serial_fd, "MRX", data_buf);
						}
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


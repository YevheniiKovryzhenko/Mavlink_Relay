#ifndef SETTINGS_HPP
#define SETTINGS_HPP
typedef struct settings_t
{
	bool enable_target;
	bool target_use_uart;
	char* target_uart_name;
	int target_baudrate;

	bool target_use_udp;
	char* target_ip;
	int target_port;
	int target_bind_port;

	bool enable_relay;
	bool relay_use_uart;
	char* relay_uart_name;
	int relay_baudrate;

	bool relay_use_udp;
	char* relay_ip;
	int relay_port;
	int relay_bind_port;

	bool enable_mocap;
	bool enable_vpe;
	bool mocap_YUP2NED;
	bool mocap_ZUP2NED;
	char* mocap_ip;
	int mocap_ID;

	bool enable_control;
	bool autotakeoff;

	bool enable_telemetry;

	bool enable_print;
	bool print_mocap;
	bool print_control;
	bool print_telemetry;
	bool print_vpe;
}settings_t;

#endif // !SETTINGS_HPP

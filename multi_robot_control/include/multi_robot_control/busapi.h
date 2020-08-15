#pragma once
#include "buscfg.h"

typedef struct bus_api bus_api_t;

/// bus api callback funcs
typedef struct
{
	void (*bus_on_msg)(short msg_type, const char * msg, unsigned int len, void * ctx);
	void (*bus_connected)(void * ctx);
	void (*bus_disconnected)(void * ctx);
}bus_api_cb_t;

BUS_API BUS_EXPORT	bus_api_t * bus_api_init(const char * ip, int port, bus_api_cb_t * cb, void * ctx);

BUS_API	BUS_EXPORT	int	bus_api_register_device(bus_api_t * api, int device_id);

BUS_API	BUS_EXPORT	int	bus_api_unregister_device(bus_api_t * api, int device_id);

BUS_API	BUS_EXPORT	int	bus_api_subscribe_device(bus_api_t * api, int subscriber_id, int device_id);

BUS_API	BUS_EXPORT	int	bus_api_unsubscribe_device(bus_api_t * api, int subscriber_id, int device_id);

BUS_API BUS_EXPORT	int bus_api_send_msg(bus_api_t * api, short msg_type, const char * msg, unsigned int len);

BUS_API BUS_EXPORT	void bus_api_free(bus_api_t * api);

BUS_API BUS_EXPORT	const char * bus_api_get_ip(bus_api_t * api);

BUS_API BUS_EXPORT	int bus_api_get_port(bus_api_t * api);

BUS_API BUS_EXPORT	const char * bus_api_get_version(bus_api_t * api);

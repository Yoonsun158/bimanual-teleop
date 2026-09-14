#pragma once
#include "bridge.h"
#include <mutex>
#include <sys/socket.h>

extern std::recursive_mutex tj_sdk_mutex;
int tj_failure(int code, const char *message);
uint64_t tj_now_ns();
void tj_hook_received(const void *data, int length);
void tj_hook_publish();
void tj_hook_send(int fd, const void *data, int length,
                  const sockaddr *address, socklen_t address_length);

#ifndef TELEMETRY_HTTP_H
#define TELEMETRY_HTTP_H

#ifdef __cplusplus
extern "C" {
#endif

/** HTTP server cổng 8080: GET /api/status (JSON cảm biến + FSM). */
void telemetry_http_start(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_HTTP_H */

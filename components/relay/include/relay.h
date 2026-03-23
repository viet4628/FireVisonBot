#pragma once

#include <stdbool.h>

/**
 * @brief Initialize the relay GPIO (water pump control).
 */
void relay_init(void);

/**
 * @brief Turn relay ON (activate water pump).
 */
void relay_on(void);

/**
 * @brief Turn relay OFF (deactivate water pump).
 */
void relay_off(void);

/**
 * @brief Check if relay is currently ON.
 */
bool relay_is_on(void);

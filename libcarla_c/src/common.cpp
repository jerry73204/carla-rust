#include "carla_c/types.h"
#include "internal.h"
#include <cstring>
#include <cstdlib>

const char* carla_error_to_string(carla_error_t error) {
    switch (error) {
        case CARLA_ERROR_NONE:
            return "No error";
        case CARLA_ERROR_CONNECTION_FAILED:
            return "Connection failed";
        case CARLA_ERROR_TIMEOUT:
            return "Operation timed out";
        case CARLA_ERROR_INVALID_ARGUMENT:
            return "Invalid argument";
        case CARLA_ERROR_NOT_FOUND:
            return "Not found";
        case CARLA_ERROR_UNKNOWN:
        default:
            return "Unknown error";
    }
}

void carla_free_string(char* str) {
    if (str) {
        free(str);
    }
}

// String list implementation (using definition from internal.h)

void carla_free_string_list(carla_string_list_t* list) {
    if (list) {
        for (size_t i = 0; i < list->count; ++i) {
            free(list->strings[i]);
        }
        free(list->strings);
        delete list;
    }
}
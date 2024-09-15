/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"


static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void os_init () {
    hal_init();
    radio_init();
    LMIC_init();
}

ostime_t os_getTime () {
    return hal_ticks();
}

osjob_t* jobs = NULL;

void os_clearCallback () {
    taskENTER_CRITICAL(&my_spinlock);
    osjob_t* prev = NULL;
    for (osjob_t* j = jobs; j != NULL; j = j->next) {
        if (prev != NULL) {
            free(prev);
        }
        prev = j;
    }
    free(prev);
    taskEXIT_CRITICAL(&my_spinlock);
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%ld: Cleared jobs\n", os_getTime());
    #endif
}

// executeTask

void os_setTimedCallback (ostime_t time, osjobcb_t cb) {
    osjob_t* j = malloc(sizeof(osjob_t));

    if (j == NULL) {
        ESP_LOGE("error", "Failed to allocate memory for job");
        exit(1);
    }

    taskENTER_CRITICAL(&my_spinlock);

    j->deadline = time;
    j->func = cb;
    j->next = NULL;

    if (jobs == NULL) {
        jobs = j;
    } else {
        osjob_t* prev = NULL;
        osjob_t* current = jobs;
        while (current != NULL && current->deadline < time) {
            prev = current;
            current = current->next;
        }
        if (prev == NULL) {
            j->next = jobs;
            jobs = j;
        } else {
            prev->next = j;
            j->next = current;
        }
    }

    if (jobs->deadline <= time) {
        hal_setTask(jobs, jobs->deadline);
    }

    taskEXIT_CRITICAL(&my_spinlock);
    #if LMIC_DEBUG_LEVEL > 1
        lmic_printf("%ld: Scheduled job %p, cb %p at %ld\n", os_getTime(), j, cb, time);
    #endif
}


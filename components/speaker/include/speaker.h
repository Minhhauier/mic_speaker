#ifndef SPEAKER_H
#define SPEAKER_H

#include <stdio.h>
#include <stdlib.h>
#include <minimp3.h>
void i2s_init(void);
void speak_vietnamese(const char *text);
void speaker_task(void *pvParameters);
void response(char *text);

#endif
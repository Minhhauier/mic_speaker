#ifndef CACHE_MANAGER_H
#define CACHE_MANAGER_H

uint32_t fnv1a_32(const char *s);
bool tts_cache_load(const char *text);
bool ensure_mp3_capacity(size_t needed);
void tts_cache_store(const char *text, const uint8_t *data, size_t len);

#endif
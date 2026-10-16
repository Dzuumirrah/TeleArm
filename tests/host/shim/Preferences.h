#pragma once
#include <Arduino.h>
#include <map>
#include <string>
#include <vector>
struct Preferences {
    static std::map<std::string, std::vector<uint8_t> > &store() { static std::map<std::string, std::vector<uint8_t> > s; return s; }
    bool begin(const char *, bool = false) { return true; }
    void end() {}
    size_t putBytes(const char *k, const void *v, size_t n) { store()[k] = std::vector<uint8_t>((const uint8_t *)v, (const uint8_t *)v + n); return n; }
    size_t getBytesLength(const char *k) { return store().count(k) ? store()[k].size() : 0; }
    size_t getBytes(const char *k, void *v, size_t n) { if (!store().count(k)) return 0; size_t m = store()[k].size() < n ? store()[k].size() : n; memcpy(v, store()[k].data(), m); return m; }
    bool remove(const char *k) { return store().erase(k) > 0; }
    bool clear() { store().clear(); return true; }
};

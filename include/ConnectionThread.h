#pragma once

#include <string>

#include "../include/ServerThread.h"

struct ConnectionThreadArgs
{
    int connfd;
    std::string name;
};

void* ConnectionThread(void* args_in);

constexpr unsigned int hash(const char* s, int off = 0)
{
    return !s[off] ? 5381 : (hash(s, off+1)*33) ^ s[off];
}

inline unsigned int hash(std::string s)
{
    return hash(s.c_str());
}

// _hash suffix for use in switch statements
constexpr unsigned int operator "" _hash(const char *s, size_t length)
{
    return hash(s);
}

#pragma once
#include "Svar.h"

struct Header{
    Header():payload_size(0){}
    int64_t timestamp_nano;
    int     topic_id, payload_size;
};

struct Package{
    Package(){}
    Header hd;
    sv::SvarBuffer payload = sv::SvarBuffer(nullptr,0);
};

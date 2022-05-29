#include <zstd.h>
#include "Svar.h"

sv::SvarBuffer zstd_compress(sv::SvarBuffer src){
    size_t const cBuffSize = ZSTD_compressBound(src.size());
    std::string dst;
    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.ptr());
    size_t const cSize = ZSTD_compress(dstp, cBuffSize, srcp, src.size(), 4);
    auto code = ZSTD_isError(cSize);
    if (code) {
        return sv::SvarBuffer(nullptr,0);
    }
    dst.resize(cSize);
    return sv::SvarBuffer(dst.c_str(),dst.size()).clone();
}

sv::SvarBuffer zstd_decompress(sv::SvarBuffer src){
    size_t const cBuffSize = ZSTD_getFrameContentSize(src.ptr(), src.size());

    if (0 == cBuffSize) {
        return sv::SvarBuffer(nullptr,0);
    }

    if (ZSTD_CONTENTSIZE_UNKNOWN == cBuffSize) {
        return sv::SvarBuffer(nullptr,0);
    }

    if (ZSTD_CONTENTSIZE_ERROR == cBuffSize) {
        return sv::SvarBuffer(nullptr,0);
    }

    std::string dst;
    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.ptr());
    size_t const cSize = ZSTD_decompress(dstp, cBuffSize, srcp, src.size());
    auto code = ZSTD_isError(cSize);
    if (code) {
        return sv::SvarBuffer(nullptr,0);
    }

    return sv::SvarBuffer(dst.c_str(),cSize).clone();
}

REGISTER_SVAR_MODULE(zstd){
    svar["zstd_compress"] = zstd_compress;
    svar["zstd_decompress"] = zstd_decompress;
}

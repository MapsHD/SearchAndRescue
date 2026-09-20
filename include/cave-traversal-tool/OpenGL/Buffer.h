#pragma once

#include <cstdint>

class Buffer

{
private:
    struct BufferIMPL;

private:
    BufferIMPL* _impl = nullptr;

public:
    explicit Buffer(const uint32_t flags, const size_t size, const void* data = nullptr);
    ~Buffer();

    [[nodiscard]] uint32_t GetID() const;
    [[nodiscard]] uint32_t GetFlags() const;
    [[nodiscard]] size_t   GetSize() const;

    void Upload(const void* data, const size_t size, const size_t offset = 0) const;
    void SetAsShaderResource(const uint32_t resource_type, const uint32_t binding, const size_t size, const size_t offset) const;
};

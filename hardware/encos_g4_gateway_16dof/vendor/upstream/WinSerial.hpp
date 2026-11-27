#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <array>
#include <stdexcept>
#include <string>
#include "GatewayProtocol.hpp"

// One owning thread. Buffers and OVERLAPPED objects live until cancellation completes.
class WinSerial {
public:
    ~WinSerial() { Close(); }
    void Open(const char* port, int baud) {
        Close();
        std::string name = std::string("\\\\.\\") + port;
        handle_ = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                              OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        if (handle_ == INVALID_HANDLE_VALUE) Fail("CreateFile");
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(handle_, &dcb)) Fail("GetCommState");
        dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE; dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE; dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = FALSE; dcb.fInX = FALSE; dcb.fAbortOnError = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
        if (!SetCommState(handle_, &dcb)) Fail("SetCommState");
        COMMTIMEOUTS timeouts{MAXDWORD, MAXDWORD, 50, 0, 50};
        if (!SetCommTimeouts(handle_, &timeouts)) Fail("SetCommTimeouts");
        if (!SetupComm(handle_, 65536, 65536)) Fail("SetupComm");
        if (!PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR)) Fail("PurgeComm");
        read_.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        write_.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        if (!read_.hEvent || !write_.hEvent) Fail("CreateEvent");
    }
    template<class Consumer> void Pump(Consumer consume) {
        for (unsigned budget = 0; budget < 64; ++budget) {
            DWORD count = 0;
            if (read_pending_) {
                if (!GetOverlappedResult(handle_, &read_, &count, FALSE)) {
                    if (GetLastError() == ERROR_IO_INCOMPLETE) return;
                    read_pending_ = false;
                    Fail("Read completion");
                }
                read_pending_ = false;
            } else {
                ResetEvent(read_.hEvent);
                if (!ReadFile(handle_, read_buffer_.data(), DWORD(read_buffer_.size()), &count, &read_)) {
                    if (GetLastError() != ERROR_IO_PENDING) Fail("ReadFile");
                    read_pending_ = true;
                    return;
                }
            }
            if (count) consume(read_buffer_.data(), count);
        }
    }
    bool WriteBusy() {
        if (!write_pending_) return false;
        DWORD count = 0;
        if (!GetOverlappedResult(handle_, &write_, &count, FALSE)) {
            if (GetLastError() == ERROR_IO_INCOMPLETE) return true;
            write_pending_ = false;
            Fail("Write completion");
        }
        write_pending_ = false;
        AdvanceWrite(count);
        return write_pending_;
    }
    bool Send(const GatewayWireFrame* frames, size_t count) {
        if (WriteBusy()) return false;
        const size_t length = count * sizeof(GatewayWireFrame);
        if (length > write_buffer_.size()) throw std::runtime_error("Serial batch too large");
        if (!length) return true;
        std::memcpy(write_buffer_.data(), frames, length);
        write_size_ = DWORD(length); write_offset_ = 0;
        StartWrite();
        return true;
    }
    void Wait(DWORD milliseconds, HANDLE wake = nullptr) {
        HANDLE events[3]; DWORD count = 0;
        if (read_pending_) events[count++] = read_.hEvent;
        if (write_pending_) events[count++] = write_.hEvent;
        if (wake) events[count++] = wake;
        if (count) WaitForMultipleObjects(count, events, FALSE, milliseconds);
        else Sleep(milliseconds);
    }
    void Close() noexcept {
        if (handle_ != INVALID_HANDLE_VALUE) {
            CancelIoEx(handle_, nullptr);
            DWORD count;
            if (read_pending_) GetOverlappedResult(handle_, &read_, &count, TRUE);
            if (write_pending_) GetOverlappedResult(handle_, &write_, &count, TRUE);
            CloseHandle(handle_);
        }
        if (read_.hEvent) CloseHandle(read_.hEvent);
        if (write_.hEvent) CloseHandle(write_.hEvent);
        handle_ = INVALID_HANDLE_VALUE; read_ = {}; write_ = {};
        read_pending_ = write_pending_ = false;
    }
private:
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    OVERLAPPED read_{}, write_{};
    std::array<uint8_t, 8192> read_buffer_{}, write_buffer_{};
    bool read_pending_ = false, write_pending_ = false;
    DWORD write_size_ = 0, write_offset_ = 0;
    [[noreturn]] static void Fail(const char* operation) {
        throw std::runtime_error(std::string(operation) + " Windows error " + std::to_string(GetLastError()));
    }
    void AdvanceWrite(DWORD count) {
        if (!count) throw std::runtime_error("Serial write made no progress");
        write_offset_ += count;
        if (write_offset_ < write_size_) StartWrite();
    }
    void StartWrite() {
        while (write_offset_ < write_size_) {
            ResetEvent(write_.hEvent);
            DWORD count = 0;
            if (!WriteFile(handle_, write_buffer_.data() + write_offset_, write_size_ - write_offset_, &count, &write_)) {
                if (GetLastError() != ERROR_IO_PENDING) Fail("WriteFile");
                write_pending_ = true;
                return;
            }
            if (!count) throw std::runtime_error("Serial write made no progress");
            write_offset_ += count;
        }
    }
};

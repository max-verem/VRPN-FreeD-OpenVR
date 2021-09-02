#include <windows.h>
#include <memory>
#include <chrono>
#include <thread>
#include "vrpn_Server_OpenVR.h"
#include "console.h"

static volatile int done = 0;
std::unique_ptr<vrpn_Server_OpenVR> server{};

// install a signal handler to shut down the devices
// On Windows, the signal handler is run in a different thread from
// the main application.  We don't want to go destroying things in
// here while they are being used there, so we set a flag telling the
// main program it is time to exit.
#if defined(_WIN32) && !defined(__CYGWIN__)
/**
* Handle exiting cleanly when we get ^C or other signals.
*/
BOOL WINAPI handleConsoleSignalsWin(DWORD signaltype)
{
    switch (signaltype) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        done = 1;
        return TRUE;
        // Don't exit, but return FALSE so default handler
        // gets called. The default handler, ExitProcess, will exit.
    default:
        return FALSE;
    }
}
#endif

static void timer_ticker_thread(HANDLE* pevent, int sleep_interval_ms)
{
    LARGE_INTEGER ticks_per_second;

    QueryPerformanceFrequency(&ticks_per_second);

    while (!done)
    {
        LARGE_INTEGER counter_before;
        QueryPerformanceCounter(&counter_before);
        LONGLONG wait_ticks = ticks_per_second.QuadPart * sleep_interval_ms / 1000LL;
        LONGLONG wait_until = counter_before.QuadPart + wait_ticks;
        for (;;)
        {
            LARGE_INTEGER counter_after;
            if (!QueryPerformanceCounter(&counter_after))
                break;
            if (counter_after.QuadPart >= wait_until)
                break;
        };
        SetEvent(*pevent);
    }
};

static void console_update_thread()
{
    while (!done)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        console_blit_fb();
    };
};

int main(int argc, char *argv[]) {
    HANDLE hTimer;
#if 0
#define WS_VER_MAJOR 2
#define WS_VER_MINOR 2

    // init winsock
    WSADATA wsaData;
    WSAStartup
    (
        ((unsigned long)WS_VER_MAJOR) |
        (((unsigned long)WS_VER_MINOR) << 8),
        &wsaData
    );
#endif
    std::thread cu(console_update_thread);
    server = std::make_unique<vrpn_Server_OpenVR>(argc, argv);
    hTimer = CreateEvent(NULL, FALSE, FALSE, NULL);
    std::thread ticker(timer_ticker_thread, &hTimer, server->sleep_interval);
    while (!done) {
        server->mainloop();
        WaitForSingleObject(hTimer, 1000);
    }
    server.reset(nullptr);
    cu.join();
    return 0;
}
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

static void console_update_thread()
{
    while (!done)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        console_blit_fb();
    };
};

int64_t qpc_diff_us(LARGE_INTEGER *B, LARGE_INTEGER *A)
{
    int64_t r;
    LARGE_INTEGER ticks_per_second;
    QueryPerformanceFrequency(&ticks_per_second);

    r = (B->QuadPart - A->QuadPart) * 1000000LL / ticks_per_second.QuadPart;

    return r;
};

int main(int argc, char *argv[]) {

    std::thread cu(console_update_thread);
    server = std::make_unique<vrpn_Server_OpenVR>(argc, argv);

    SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    LARGE_INTEGER counter_to, counter_now, ticks_per_second, counter_overrun;
    QueryPerformanceCounter(&counter_to);

#define MAX_HIST_OVERRUN 32
    int64_t overruns_cnt = 0, overruns1_last, overruns2_last;
    int64_t overruns1[MAX_HIST_OVERRUN], overruns2[MAX_HIST_OVERRUN];

    while (!done) {
        QueryPerformanceFrequency(&ticks_per_second);
        QueryPerformanceCounter(&counter_now);
        counter_to.QuadPart += ticks_per_second.QuadPart * server->sleep_interval / 1000LL;
        if (counter_to.QuadPart < counter_now.QuadPart)
        {
            int64_t t1 = qpc_diff_us(&counter_now, &counter_to);
            counter_to.QuadPart = counter_now.QuadPart;
        }
        else
        for (;;)
        {
            if (!QueryPerformanceCounter(&counter_now))
                break;
            if (counter_now.QuadPart >= counter_to.QuadPart)
                break;
        };
        overruns1_last = overruns1[overruns_cnt % MAX_HIST_OVERRUN] = qpc_diff_us(&counter_now, &counter_to);
        server->mainloop();
        QueryPerformanceCounter(&counter_overrun);
        overruns2_last = overruns2[overruns_cnt % MAX_HIST_OVERRUN] = qpc_diff_us(&counter_overrun, &counter_now);
        overruns_cnt++;
    }
    server.reset(nullptr);
    cu.join();
    return 0;
}
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <mutex>
#include "console.h"

char console_fb[3][console_window_height][console_window_width];
static int console_window_row = 0;

char console_keypress(HANDLE hStdin)
{
    DWORD ne = 0;
    if (GetNumberOfConsoleInputEvents(hStdin, &ne) && ne)
    {
        INPUT_RECORD irInBuf[128];
        DWORD cNumRead;
        if (ReadConsoleInput(
            hStdin,      // input buffer handle 
            irInBuf,     // buffer to read into 
            128,         // size of read buffer 
            &cNumRead)) // number of records read 
        {
            if(cNumRead && irInBuf[0].EventType == KEY_EVENT)
                return irInBuf[0].Event.KeyEvent.uChar.AsciiChar;
        }
    }
    return 0;
}

void console_setup(HANDLE *p_c_in, HANDLE *p_c_out)
{
    BOOL b;
    DWORD e = 0;
    HANDLE h_console_input, h_console_output;

    *p_c_in = h_console_input = GetStdHandle(STD_INPUT_HANDLE);
    *p_c_out = h_console_output = GetStdHandle(STD_OUTPUT_HANDLE);

//    SetStdHandle(STD_ERROR_HANDLE, GetStdHandle(STD_OUTPUT_HANDLE));

    // enable window input
    b = SetConsoleMode(h_console_input, ENABLE_WINDOW_INPUT);
    if (!b) e = GetLastError();

    SMALL_RECT const minimal_window = { 0, 0, 1, 1 };
    b = SetConsoleWindowInfo(h_console_output, TRUE, &minimal_window);
    if (!b) e = GetLastError();

    COORD const size = { console_window_width, console_window_height };
    b = SetConsoleScreenBufferSize(h_console_output, size);
    if (!b) e = GetLastError();

    SMALL_RECT const window = { 0, 0, size.X - 1, size.Y - 1 };
    b = SetConsoleWindowInfo(h_console_output, TRUE, &window);
    if (!b) e = GetLastError();

    console_cls(h_console_output);
}


// https://docs.microsoft.com/uk-ua/windows/console/clearing-the-screen
void console_cls(HANDLE hConsole)
{
    COORD coordScreen = { 0, 0 };    // home for the cursor 
    DWORD cCharsWritten;
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    DWORD dwConSize;

    // Get the number of character cells in the current buffer. 

    if (!GetConsoleScreenBufferInfo(hConsole, &csbi))
    {
        return;
    }

    dwConSize = csbi.dwSize.X * csbi.dwSize.Y;

    // Fill the entire screen with blanks.

    if (!FillConsoleOutputCharacter(hConsole,        // Handle to console screen buffer 
        (TCHAR) ' ',     // Character to write to the buffer
        dwConSize,       // Number of cells to write 
        coordScreen,     // Coordinates of first cell 
        &cCharsWritten))// Receive number of characters written
    {
        return;
    }

    // Get the current text attribute.

    if (!GetConsoleScreenBufferInfo(hConsole, &csbi))
    {
        return;
    }

    // Set the buffer's attributes accordingly.

    if (!FillConsoleOutputAttribute(hConsole,         // Handle to console screen buffer 
        csbi.wAttributes, // Character attributes to use
        dwConSize,        // Number of cells to set attribute 
        coordScreen,      // Coordinates of first cell 
        &cCharsWritten)) // Receive number of characters written
    {
        return;
    }

    // Put the cursor at its home coordinates.

    SetConsoleCursorPosition(hConsole, coordScreen);
}

void console_put(char* str)
{
    char fmt[16];

    if (console_window_row >= console_window_height)
        return;

    snprintf(fmt, sizeof(fmt), "%%-%ds", console_window_width - 2);
    snprintf(console_fb[0][console_window_row++], console_window_width, fmt, str);
}

int vscprintf(const char *format, va_list ap)
{
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int retval = vsnprintf(NULL, 0, format, ap_copy);
    va_end(ap_copy);
    return retval;
}

int vasprintf(char **strp, const char *format, va_list ap)
{
    int len = vscprintf(format, ap);
    if (len == -1)
        return -1;
    char *str = (char*)malloc((size_t)len + 1);
    if (!str)
        return -1;
    int retval = vsnprintf(str, len + 1, format, ap);
    if (retval == -1) {
        free(str);
        return -1;
    }
    *strp = str;
    return retval;
}

int asprintf(char **strp, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int retval = vasprintf(strp, format, ap);
    va_end(ap);
    return retval;
}

static std::mutex g_pages_mutex;

void console_swap_fb()
{
    std::lock_guard<std::mutex> guard(g_pages_mutex);

    memcpy(console_fb[1], console_fb[0], console_window_width * console_window_height);

    // reset counter
    console_window_row = 0;

    // clear
    memset(console_fb[0], 0, console_window_width * console_window_height);
};

void console_blit_fb()
{
    {
        std::lock_guard<std::mutex> guard(g_pages_mutex);
        memcpy(console_fb[2], console_fb[1], console_window_width * console_window_height);
    };

    SetConsoleCursorPosition(console_out, { 0, 0 });
    //    console_cls(GetStdHandle(STD_ERROR_HANDLE));
    //    console_cls(GetStdHandle(STD_OUTPUT_HANDLE));
    for (int i = 0; i < console_window_height - 1; i++)
        fprintf(stdout, "%s\n", console_fb[2][i]);
};

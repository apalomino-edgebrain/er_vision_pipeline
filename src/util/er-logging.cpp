//#############################################################################
                                 /*-:::--.`
                            -+shmMMNmmddmmNMNmho/.
                 `yyo++osddMms:                  `/yNNy-
              yo    +mMy:                       `./dMMdyssssso-
              oy  -dMy.                     `-+ssso:.`:mMy`   .ys
                ho+MN:                  `:osso/.         oMm-   +h
                +Mmd-           `/syo-                     :MNhs`
                `NM-.hs`      :syo:                          sMh
                oMh   :ho``/yy+.                             `MM.
                hM+    `yNN/`                                 dM+
                dM/  -sy/`/ho`                                hMo
                hMo/ho.     :yy-                             dM/
            :dNM/             :yy:                         yMy
            sy`:MN.              `+ys-                     +Mm`
            oy`   :NM+                  .+ys/`           `hMd.ys
            /sssssyNMm:                   `:sys:`     `oNN+   m-
                        .sNMh+.                   `:sNMdyysssssy:
                        -odMNhs+:-.`    `.-/oydMNh+.
                            `-+shdNMMMMMMMNmdyo/.
                                    `````*/
//#############################################################################
// Basic tools to print with pretty logging
//#############################################################################

#include "../er-pipeline.h"

//-------------------------------------------------------------------------------
enum STR_PAD { STR_PAD_RIGHT, STR_PAD_LEFT, STR_PAD_BOTH };

void print_pad(const char *str, uint8_t len = 0, const char *pad = " ", STR_PAD dir = STR_PAD_RIGHT)
{
    int padding;
    int slen = int(strlen(str));
    if (len + 1 >= slen) {
        switch (dir) {
            case STR_PAD_LEFT:
                padding = len - slen - 1;
                for (int c = 0; c < padding; c++)
                    printf("%s", pad);

                printf(" %s\n", str);
                break;

            case STR_PAD_BOTH:
                if (slen > 0)
                    slen += 2;

                padding = (len - slen) / 2;
                for (int c = 0; c < padding; c++)
                    printf("%s", pad);
                if (slen > 0)
                    printf(" %s ", str);

                padding = len - slen - padding;
                for (int c = 0; c < padding; c++)
                    printf("%s", pad);

                printf("\n");
                break;

            default:
                padding = len - slen;
                printf("%s ", str);
                for (int c = 0; c < padding; c++)
                    printf("%s", pad);
                printf("\n");
                break;
        }
    } else {
        printf("%s\n", str);
    }
}

#define TEXT_PADDING 40

void clear_screen()
{
    printf("\033[2J\033[;H");
}

void printf_h1(const char* szOutput, ...)
{
    char		out[256];
    va_list		va;

    va_start(va, szOutput);
    vsprintf(out, szOutput, va);
    va_end(va);

    printf("\033[40;1H");
    print_pad("", TEXT_PADDING, "#", STR_PAD_BOTH);
    print_pad(out, TEXT_PADDING, "#", STR_PAD_BOTH);
    print_pad("", TEXT_PADDING, "#", STR_PAD_BOTH);
}

void printf_h2(const char* szOutput, ...)
{
    char		out[256];
    va_list		va;

    va_start(va, szOutput);
    vsprintf(out, szOutput, va);
    va_end(va);

    print_pad(out, TEXT_PADDING, "-", STR_PAD_BOTH);
}

void printf_(const char *header, const char* szOutput, ...)
{
    char		out[256];
    va_list		va;

    va_start(va, szOutput);
    vsprintf(out, szOutput, va);
    va_end(va);

    printf("%s", header);
    print_pad(out, uint8_t(TEXT_PADDING - strlen(header)), " ", STR_PAD_LEFT);
}

//-------------------------------------------------------------------------------
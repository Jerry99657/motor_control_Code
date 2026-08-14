#include "app_context.h"
#include <string.h>

static AppContext s_app_context;

void AppContext_Init(const AppContext *context)
{
    if (context == NULL)
    {
        memset(&s_app_context, 0, sizeof(s_app_context));
        return;
    }

    s_app_context = *context;
}

const AppContext *AppContext_Get(void)
{
    return &s_app_context;
}

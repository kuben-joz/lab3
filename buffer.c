#include "buffer.h"
#include "string.h"

/*
typedef struct buffer {
    int start;
    int end;
    int buff[BUFF_SIZE];
} buffer;
*/

void buff_init(buffer *buff)
{
    buff->start = 0;
    buff->end = 0;
}

void buff_push(buffer *buff, int val)
{
    buff->buff[buff->end++] = val;
    buff->end %= BUFF_SIZE;
}

int buff_pop(buffer *buff)
{
    int res = buff->buff[buff->start++];
    buff->start %= BUFF_SIZE;
    return res;
}

/*
int buff_full(buffer* buff) {
    return buff->start == (buff->end + 1) % BUFF_SIZE;
}
*/

int buff_empty(buffer *buff)
{
    return buff->start == buff->end;
}
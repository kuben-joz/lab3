#define BUFF_SIZE 32

typedef struct buffer
{
    int start;
    int end;
    int buff[BUFF_SIZE];
} buffer;

void buff_init(buffer *buff);

void buff_push(buffer *buff, int val);

int buff_pop(buffer *buff);

int buff_full(buffer *buff);

int buff_empty(buffer *buff);

#ifndef   __QUEUE_H__
#define   __QUEUE_H__

#ifdef __QUEUE_C__
	#define QUEUE_EXT
#else
	#define QUEUE_EXT extern
#endif

#define BUFF_SIZE 256

typedef struct Que
{
    char buff[BUFF_SIZE];
    int head;
    int tail;
    int size;
}Que_t;

QUEUE_EXT Que_t rxQue[3],txQue[3];

QUEUE_EXT void Que_Clear(Que_t* pQue);
QUEUE_EXT char Que_PutByte(Que_t* pQue, char data);
QUEUE_EXT char Que_GetByte(Que_t* pQue, char *data);
QUEUE_EXT int Que_GetSize(Que_t* pQue);
#endif

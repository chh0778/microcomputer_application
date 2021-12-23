#define __QUEUE_C__
    #include "Que.h"
#undef  __QUEUE_C__


void Que_Clear(Que_t* pQue)
{
    pQue->head = pQue->tail = pQue->size = 0;
    memset(pQue->buff,0,BUFF_SIZE);
}

char Que_PutByte(Que_t* pQue, char data)
{
    if(Que_GetSize(pQue) == (BUFF_SIZE-1)) return 0;
    pQue->buff[pQue->head++] =data;
    pQue->head %= BUFF_SIZE;
    
    pQue->size = Que_GetSize(pQue);
    return 1;
}

char Que_GetByte(Que_t* pQue, char *data)
{
    if(Que_GetSize(pQue) == 0) return 0;
    *data = pQue->buff[pQue->tail++];
    pQue->tail %= BUFF_SIZE;
    
    pQue->size = Que_GetSize(pQue);
    return 1;
}

int Que_GetSize(Que_t* pQue)
{
    return (pQue->head - pQue->tail  + BUFF_SIZE) % BUFF_SIZE;
}



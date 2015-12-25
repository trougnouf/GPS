/*

  $Id: dll_hj.h,v 1.1 2011/07/31 03:54:40 cwkelley Exp $
  
  $Log: dll_hj.h,v $
  Revision 1.1  2011/07/31 03:54:40  cwkelley
  added a doubly linked list to make program more robust when other processes
  are running on the computer

  Revision 1.1.1.1  2010/03/30 15:33:46  jones
  Initial import of sources


*/
#ifndef _DLL_HJ_H
#define _DLL_HJ_H

/*
#define DLLDEBUG 1
*/

/* Heiko's Special 1 Producer/N Consumer pthread Double Linked List Routines */
/* Version: dll_hj.h,v 1.4 2003/06/19 17:44:05 jones Exp */

/* These routines will enable multiple threads to share streamed data */
/* safely in a 1 thread Producer/ N thread Consumer model. */

/* For Tru64 Unix */
#ifdef __DECC
#pragma nomember_alignment
#endif

/* Typedefs, and Declarations */
typedef struct node
{
  char           *oktodel;
  struct node    *next;
  struct node    *prior;
  void           *info;
  pthread_mutex_t oktodelmutex;
  unsigned int    infosize;
}
Node;

typedef struct list
{
  unsigned long long listsize;
  unsigned long long listbytes;
  Node           *head;
  Node           *tail;
  int             numclients;
}
List;

/******************************  dll routines ****************************/
List           *
dll_newlist(List ** list, int numclients);
void
dll_dellist(List ** list);
unsigned long long
dll_listsize(List * list);
unsigned long long
dll_listbytes(List * list);
Node           *
dll_addnode(List * list, void *info, unsigned int infosize);
int
dll_delnode(List * list, Node * node);
int
dll_readoktodel(List * list, Node * node, int index);
int
dll_tryreadoktodel(List * list, Node * node, int index);
void
dll_writeoktodel(List * list, Node * node, int index, int value);
void           *
dll_infonode(List * list, Node * node);
int
dll_delallnodes(List * list);
int
dll_markallnodes(List * list, int index, int value);
int
dll_delmarked(List * list, unsigned char *dellist);
int
dll_delmarkeduntilnotok(List * list, unsigned char *dellist);
int
dll_trimlist_fifo(List * list, char *dellist, unsigned long long bytes);
int
dll_printlist(List * list, int flag);
int
dll_printnode(List * list, Node * node, int flag);

#endif /* _DLL_HJ_H */

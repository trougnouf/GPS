/*

* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */ 


/*
  $Id: dll_hj.c,v 1.1 2011/07/31 03:54:40 cwkelley Exp $
  
  $Log: dll_hj.c,v $
  Revision 1.1  2011/07/31 03:54:40  cwkelley
  added a doubly linked list to make program more robust when other processes
  are running on the computer

  Revision 1.1.1.1  2010/03/30 15:33:46  jones
  Initial import of sources


*/
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <limits.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/un.h>
#include <time.h>

#include "dll_hj.h"

/*
#define DLLDEBUG 1
*/

/* Heiko's Special 1 Producer/N Consumer pthread Double Linked List Routines */
/* Version: dll_hj.h,v 1.4 2003/06/19 17:44:05 jones Exp */

/* These routines will enable multiple threads to share streamed data */
/* safely in a 1 thread Producer/ N thread Consumer model. */

/******************************  dll routines ****************************/
List           *
dll_newlist(List ** list, int numclients)
{
  /* Calloc List struct so that all values are zeroed out... */
  if ((*list = (List *) calloc(sizeof(List), 1)) == NULL)
    return NULL;

  (*list)->listsize = 0L;
  (*list)->listbytes = 0L;
  (*list)->numclients = numclients;

#ifdef DLLDEBUG
  printf(" New dll list with %d clients\n", (*list)->numclients);
#endif
  return (*list);
}

void
dll_dellist(List ** list)
{
  extern int      dll_delallnodes(List * list);

  if (*list == NULL)
    return;

  dll_delallnodes(*list);
  free(*list);
  *list = NULL;
}

unsigned long long
dll_listsize(List * list)
{
  return list->listsize;
}

unsigned long long
dll_listbytes(List * list)
{
  return list->listbytes;
}

Node           *
dll_addnode(List * list, void *info, unsigned int infosize)
{
  Node           *newnode, *oldtail;
  int             i;

  /* Allocate space for new node */
  if ((newnode = (Node *) malloc(sizeof(Node))) == NULL)
    return NULL;

  /* Allocated space for delete semephores to nodes */
  if ((newnode->oktodel = (char *) calloc(sizeof(char), list->numclients)) == NULL)
    return NULL;

  /* Init the oktodel mutex for default fast mutex... */
  if ((i = pthread_mutex_init(&newnode->oktodelmutex, NULL)) != 0) {
    printf(" Problem node pthread_mutex_init %d %s\n", i, strerror(i));
    return NULL;
  }

  /* If list->head is NULL, assume empty list and this is the 1st record. */
  if (list->head == NULL) {
    newnode->infosize = infosize;
    newnode->next = NULL;
    newnode->prior = NULL;
    newnode->info = info;
    for(i = 0;i < list->numclients;i++) {
      newnode->oktodel[i] = 0;
    }
    list->head = newnode;
    list->tail = newnode;
    list->listsize = 1L;
    list->listbytes = (infosize + sizeof(Node));
    return newnode;
  }

  /* Save old tail in local var so we can update it's next pointer last */
  oldtail = list->tail;

  for(i = 0;i < list->numclients;i++)
    newnode->oktodel[i] = 0; 
  newnode->infosize = infosize;
  newnode->next = NULL;
  newnode->prior = oldtail;
  newnode->info = info;
  list->listsize++;
  list->listbytes += (infosize + sizeof(Node));
  list->tail = newnode;
  /* Change the next pointer to new tail the very last! */
  oldtail->next = newnode;

  return newnode;
}

int
dll_delnode(List * list, Node * node)
{
  int i;
  
  /* This is the first node. */
  if (node == list->head) {
    if (node->next != NULL)
      node->next->prior = NULL;
    list->head = node->next;
  }
  /* This is the last node. */
  else if (node == list->tail) {
    node->prior->next = NULL;
    list->tail = node->prior;
  }
  /* This is a middle node. */
  else {
    node->prior->next = node->next;
    node->next->prior = node->prior;
  }

  /* Destroy the oktodel mutex... */
  if ((i = pthread_mutex_destroy(&node->oktodelmutex)) != 0) {
    printf(" Problem node 0x%08x oktodel pthread_mutex_destroy %d %s\n", (unsigned int)node, i, strerror(i));
  }
  list->listbytes -= (node->infosize + sizeof(Node));
#ifdef DLLDEBUG1
  printf(" dll_delnode:free %p\n", node);
#endif
  free(node->oktodel);
  free(node->info);
  free(node);
  list->listsize--;
  return 1;
}

int
dll_readoktodel(List * list, Node * node, int index)
{
  int i;
  pthread_mutex_lock(&node->oktodelmutex);
  i = node->oktodel[index];
  pthread_mutex_unlock(&node->oktodelmutex);
  return i;
}

int
dll_tryreadoktodel(List * list, Node * node, int index)
{
  int i;
  if(pthread_mutex_trylock(&node->oktodelmutex) != EBUSY) {
    i = node->oktodel[index];
    pthread_mutex_unlock(&node->oktodelmutex);
  }
  else {
    return -1;
  }
  return i;
}

void
dll_writeoktodel(List * list, Node * node, int index, int value)
{
  pthread_mutex_lock(&node->oktodelmutex);
  node->oktodel[index] = value;
  pthread_mutex_unlock(&node->oktodelmutex);
  return;
}

void           *
dll_infonode(List * list, Node * node)
{
  return (node->info);
}


int
dll_delallnodes(List * list)
{
  void           *info;
  Node           *node;
  int             i;
  
#ifdef DLLDEBUG
  printf(" Deleting all %lld nodes...\n",list->listsize);
#endif

  if (list->head == NULL)
    return 1;

  while (list->head != NULL) {
    info = list->head->info;
    node = list->head;
    list->head = list->head->next;
    /* Destroy the oktodel mutex... */
    if ((i = pthread_mutex_destroy(&node->oktodelmutex)) != 0) {
      printf(" Problem node 0x%08x oktodel pthread_mutex_destroy %d %s\n", (unsigned int)node, i, strerror(i));
    }
    free(node->oktodel);
    free(info);
    free(node);
  }

  list->head = NULL;
  list->tail = NULL;
  list->listsize = 0L;
  list->listbytes = 0L;
  return 1;
}

int
dll_markallnodes(List * list, int index, int value)
{
  Node           *node, *next;
  
#ifdef DLLDEBUG
  printf(" Marking oktodel all %lld nodes...\n",list->listsize);
#endif
  if (list->head == NULL)
    return 1;

  node = list->head;
  while (node != NULL) {
    next = node->next;
    dll_writeoktodel(list, node, index, value);
    node = next;
  }

  return 1;
}

int
dll_delmarked(List * list, unsigned char *dellist)
{
  Node           *node, *next = NULL;
  int             i, delflag = 0, result;
  
  
#ifdef DLLDEBUG
  printf("\n dll_delmarked:%lld head:0x%08x\n",list->listsize,(unsigned int)list->head);
#endif
  /*  Move the clientptr to first one not processed and reset readptr flags */
  node = list->head;
  while (node != NULL) {
    next = node->next;
    delflag = 1;
#ifdef DLLDEBUG
    printf(" dll_delmarked: node:0x%08x\n", (unsigned int)node);
#endif
    for(i = 0; i < list->numclients; i++) {
      result = dll_tryreadoktodel(list, node, i);
#ifdef DLLDEBUG
      printf(" dll_delmarked: i:%d result:%d dellist[%d]:%d\n",i,result,i,dellist[i]);
#endif
      /* Check oktodel[i] for this node against the dellist[i] */
      if(result < dellist[i])
        delflag = 0;
    }
    if(delflag == 1) {
#ifdef DLLDEBUG
      printf(" dll_delmarked: del:0x%08x\n", (unsigned int)node);
#endif
      dll_delnode(list, node);
    }
    node = next;
  }
#ifdef DLLDEBUG
  printf("  dll_delmarked: done\n");
#endif
  return 1;
}

int
dll_delmarkeduntilnotok(List * list, unsigned char *dellist)
{
  Node           *node, *next = NULL;
  int             i, delflag = 0, result;
  
  
#ifdef DLLDEBUG1
  printf("\n dll_delmarkeduntilnotok:%lld head:0x%08x\n",list->listsize,(unsigned int)list->head);
#endif
  /*  Move the clientptr to first one not processed and reset readptr flags */
  node = list->head;
  while (node != NULL) {
    next = node->next;
    delflag = 1;
#ifdef DLLDEBUG1
    printf(" dll_delmarkeduntilnotok: node:0x%08x\n", (unsigned int)node);
#endif
    for(i = 0; i < list->numclients; i++) {
      result = dll_tryreadoktodel(list, node, i);
#ifdef DLLDEBUG1
      printf(" dll_delmarkeduntilnotok: i:%d result:%d dellist[%d]:%d\n",i,result,i,dellist[i]);
#endif
      
      /* Check oktodel[i] for this node against the dellist[i] */
      if(result < dellist[i]) {
        delflag = 0;
        next = NULL;
        break;
      }
    }
    if(delflag == 1) {
#ifdef DLLDEBUG1
      printf(" dll_delmarkeduntilnotok: del:0x%08x\n", (unsigned int)node);
#endif
      dll_delnode(list, node);
    }
    node = next;
  }
#ifdef DLLDEBUG1
  printf("  dll_delmarkeduntilnotok: done\n");
#endif
  return 1;
}

int
dll_trimlist_fifo(List * list, char *dellist, unsigned long long bytes)
{
  Node           *node, *next = NULL;
  int             i, j, delflag = 0, result;


  /* get the first node in the list to start with */
  node = list->head;
  j = 0;
  while (list->listbytes > bytes) {
    next = node->next;
    delflag = 1;
    for (i = 0; i < list->numclients; i++) {
      result = dll_tryreadoktodel(list, node, i);
      /* Check oktodel[i] for this node against the dellist[i] */
      if (result < dellist[i])
	delflag = 0;
    }
    if (delflag == 1) {
      dll_delnode(list, node);
      j++;
    }
    node = next;
  }
  return j;
}

/* Delete a block of nodes of amount size from end of list */
int
dll_trimnodes_fifo(List * list, char *dellist, unsigned long long size)
{
  Node           *node, *next = NULL;
  int             i, j, delflag = 0, result;
  
  
  /* get the first node in the list to start with */
  node = list->head;
	j = 0;
  while (list->listsize > size && node != NULL) {
    next = node->next;
    delflag = 1;
    for(i = 0; i < list->numclients; i++) {
      result = dll_tryreadoktodel(list, node, i);
      /* Check oktodel[i] for this node against the dellist[i] */
      if(result < dellist[i])
        delflag = 0;
    }
    if(delflag == 1) {
      dll_delnode(list, node);
			j++;
    }
    node = next;
  }
  return j;
}

int
dll_printlist(List * list, int flag)
{
  Node           *node;
  int             i, j = 0;
  unsigned char  *c;

  printf("******************* Print dll **********************\n");
  printf("list:           0x%08x\n", (unsigned int) list);
  printf("list size:      %lld\n", list->listsize);
  printf("list bytes:      %lld\n", list->listbytes);
  printf("list head:      0x%08x\n", (unsigned int) list->head);
  printf("list tail:      0x%08x\n", (unsigned int) list->tail);

  if (list->head == NULL)
    return 1;

  node = list->head;
  while (node != NULL) {
    printf("node number:    %d\n", j++);
    printf(" node address:    0x%08x\n", (unsigned int) node);
    for (i = 0; i < list->numclients; i++)
      printf(" node->oktodel[%d]:0x%02x\n", i, dll_readoktodel(list, node, i));
    printf(" node->infosize:  %d\n", node->infosize);
    printf(" node->next:      0x%08x\n", (unsigned int) node->next);
    printf(" node->prior:     0x%08x\n", (unsigned int) node->prior);
    printf(" node->info:      0x%08x\n", (unsigned int) node->info);
    if(flag == 1) {
      printf(" node->info:      ");
      c = (unsigned char *) node->info;
      for (i = 0; i < (int)node->infosize; i++) {
        printf("%02x", (unsigned char) c[i]);
        if (!((i + 1) % 16))
	  printf("\n                  ");
      }
      printf("\n");
    }
    else if(flag == 2) {
      printf(" node->info:      ");
      c = (unsigned char *) node->info;
      for (i = 0; i < 16; i++) {
        printf("%02x", (unsigned char) c[i]);
      }
      printf("\n");
    }
    node = node->next;
  }
  return 1;
}

int
dll_printnode(List * list, Node * node, int flag)
{
  int             i;
  unsigned char  *c;
  
  printf("***************** Print dll node *******************\n");
  printf(" node address:    0x%08x\n", (unsigned int) node);
  for (i = 0; i < list->numclients; i++)
    printf(" node->oktodel[%d]:0x%08x\n", i, dll_readoktodel(list, node, i));
  printf(" node->infosize:  %d\n", node->infosize);
  printf(" node->next:      0x%08x\n", (unsigned int) node->next);
  printf(" node->prior:     0x%08x\n", (unsigned int) node->prior);
  printf(" node->info:      0x%08x\n", (unsigned int) node->info);
  if(flag == 1) {
    printf(" node->info:      ");
    c = (unsigned char *) node->info;
    for (i = 0; i < (int) node->infosize; i++) {
      printf("%02x", (unsigned char) c[i]);
      if (!((i + 1) % 16))
	printf("\n                  ");
    }
    printf("\n");
  }
  else if(flag == 2) {
    printf(" node->info:      ");
    c = (unsigned char *) node->info;
    for (i = 0; i < 16; i++) {
      printf("%02x", (unsigned char) c[i]);
    }
    printf("\n");
  }
  printf(" node bytes:      %d\n", sizeof(Node) + node->infosize);
  return 1;
}

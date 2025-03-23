/*
 * menu.h
 *
 *  Created on: Feb 24, 2025
 *      Author: User123
 */

#ifndef MENU_H_
#define MENU_H_

/* Exported includes -----------------------------------------------*/
#include "common.h"

/* Exported defines ------------------------------------------------*/
#define MENU_MAX_NODES	5
#define MENU_UPDATE_SCREEN_TICK	100
#define MENU_UPDATE_MODEL_TICK	200

/* Exported macros -------------------------------------------------*/
#define ToMenuHandle(node) ((MenuHandle_TypeDef *)node->pvData)

/* Exported typedefs ------------------------------------------------*/
struct GenericTreeNode_
{
	// Data
	void *pvData;

	// GenericTree structure
	struct GenericTreeNode_* parent;     	// Nodo padre
	struct GenericTreeNode_* nextSibling;	// Nodo siguiente
	struct GenericTreeNode_* prevSibling;   // Nodo anterior
	struct GenericTreeNode_* firstChild;    // Nodo hijo
};

typedef struct GenericTreeNode_ GTreeNode;
typedef GTreeNode **GTreeNodeHandle;
typedef struct
{
	void (*draw)(GTreeNodeHandle);          	// Función callback para dibujar en la LCD
	void (*action)(GTreeNodeHandle);        	// Función callback para la acción asociada
	void *pvParam;
} MenuHandle_TypeDef;

typedef enum
{
	GTREE_INSERT_CHILD = 0,
	GTREE_INSERT_SIBLING
} GTreeFlags_t;

/* Exported variables -----------------------------------------------*/

/* Exported prototype function --------------------------------------*/
// Generic tree data structure API
int GTree_Goto_Next(GTreeNodeHandle root);
int GTree_Goto_Prev(GTreeNodeHandle root);
int GTree_Goto_Child(GTreeNodeHandle root);
int GTree_Goto_Parent(GTreeNodeHandle root);
int GTree_Insert(GTreeNodeHandle root, GTreeNode *node, GTreeFlags_t flags);

// Menu API
GTreeNode *Menu_Insert(GTreeNodeHandle root, GTreeFlags_t flags, void (*draw)(GTreeNodeHandle), void (*action)(GTreeNodeHandle), void *pvParam);
// For debug only
GTreeNode *Menu_GetCurrentNode(void);

void Menu_PaintEvent(void);
void Menu_ModelEvent(void);

#endif /* MENU_H_ */

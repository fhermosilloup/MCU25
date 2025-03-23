/*
 * menu.c
 *
 *  Created on: Feb 24, 2025
 *      Author: User123
 */

#include "menu.h"

static GTreeNode xMenuNodes[MENU_MAX_NODES];
static MenuHandle_TypeDef xMenuHandler[MENU_MAX_NODES];

static uint8_t ucMenuNodeCount = 0;
static GTreeNode *pxCurrentNode = NULL;
static GTreeNode *pxCurrentParent = NULL;


GTreeNode *Menu_GetCurrentNode(void)
{
	return pxCurrentNode;
}

int GTree_Goto_Next(GTreeNodeHandle root)
{
    if (!((*root)->nextSibling)) return -1;

    *root = (*root)->nextSibling;

    return 0;
}

stm32_err_t GTree_Goto_Prev(GTreeNodeHandle root)
{
    if (!((*root)->prevSibling)) return -1;

    *root = (*root)->prevSibling;

    return 0;
}

stm32_err_t GTree_Goto_Child(GTreeNodeHandle root)
{
    if (!((*root)->firstChild)) return -1;

    *root = (*root)->firstChild;

    return 0;
}

stm32_err_t GTree_Goto_Parent(GTreeNodeHandle root)
{
    if (!((*root)->parent)) return -1;

    *root = (*root)->parent;

    return 0;
}

stm32_err_t GTree_Insert(GTreeNodeHandle root, GTreeNode *node, GTreeFlags_t flags)
{
	if(!node) return -1;

	// GTree init
	node->parent = NULL;
	node->firstChild = NULL;
	node->nextSibling = NULL;
	node->prevSibling = NULL;

	if(!*root) *root = node;
	else
	{
		// Insertion procedure
		if(flags == GTREE_INSERT_CHILD)
		{
			// Insert child
			node->parent = *root;

			// First child?
			if (!(*root)->firstChild)
			{
				(*root)->firstChild = node;
			}
			else
			{
				// Search for the last sibling
				GTreeNode *child = (*root)->firstChild;
				while(child->nextSibling)
				{
					child = child->nextSibling;
				}
				node->prevSibling = child;
				child->nextSibling = node;
			}
		}
		else
		{
			// Insert sibling
			node->parent = (*root)->parent;

			// First sibling?
			if (!(*root)->nextSibling)
			{
				node->prevSibling = *root;
				(*root)->nextSibling = node;
			}
			else
			{
				// Search for the last sibling
				GTreeNode *sibling = (*root)->nextSibling;
				while(sibling->nextSibling)
				{
					sibling = sibling->nextSibling;
				}
				node->prevSibling = sibling;
				sibling->nextSibling = node;
			}
		}
	}

	return 0;
}

// Función para agregar un nuevo nodo de menú
GTreeNode *Menu_Insert(GTreeNodeHandle hroot, GTreeFlags_t flags, void (*draw)(GTreeNodeHandle), void (*action)(GTreeNodeHandle), void *pvParam)
{
	// Verificar si hay espacio suficiente en el arreglo
    if (ucMenuNodeCount > MENU_MAX_NODES-1) return (GTreeNode *)NULL;

    // Asignar el nodo desde el arreglo estático
    GTreeNode *node = &xMenuNodes[ucMenuNodeCount];
    MenuHandle_TypeDef *hmenu = &xMenuHandler[ucMenuNodeCount];

    // Handler
    hmenu->action = action;
    hmenu->draw = draw;
    hmenu->pvParam = pvParam;
    node->pvData = hmenu;

    // Insert
    if(!hroot)
    {
    	if(!pxCurrentParent) pxCurrentNode = node;
		if(GTree_Insert(&pxCurrentParent, node, flags) != STM32_OK) return (GTreeNode *)NULL;
    }
    else
    {
    	if(GTree_Insert(hroot, node, flags) != STM32_OK) return (GTreeNode *)NULL;
    }

    // Increment node count
    ucMenuNodeCount++;

    return node;  // Retornar el índice del nodo agregado
}


void Menu_PaintEvent(void)
{
    if (pxCurrentNode)
    {
    	if(ToMenuHandle(pxCurrentNode)->draw) ToMenuHandle(pxCurrentNode)->draw(&pxCurrentNode);
    }
}

void Menu_ModelEvent(void)
{
	if(ToMenuHandle(pxCurrentNode)->action)
	{
		ToMenuHandle(pxCurrentNode)->action(&pxCurrentNode);
	}
}

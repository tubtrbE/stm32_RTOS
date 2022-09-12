#include "stdio.h"
#include "stdlib.h"

typedef struct listNode {
	int Data;
	struct listNode* Next;
	struct listNode* Prev;
}Node;

Node* createNode(int data) {

	// Node struct ��ŭ�� ������ ���� Node �� �ּҸ� ����
	Node* newNode = (Node*)malloc(sizeof(Node));

	// ���� �ʱ�ȭ 
	newNode->Data = data;
	newNode->Next = NULL;
	newNode->Prev = NULL;

	return newNode;
}

void deleteNode(Node* Node) {
	free(Node);
}

Node* getNodeAt(Node* Start, int index) {
	Node* scout = Start;
	int count = 0;

	//Node �� �ּҰ� ��� �ִٸ� stop �׷��� ������ ����ؼ� ����.
	while (scout != NULL) {
	
		//���࿡ count �� index �� �������ٸ� �������� return �������ν� �Լ��� ������.
		if (count++ == index) {
			return scout;
		}
		scout = scout->Next;
	}

	//while loop �� ��� �������� �������� ���ߴٸ� ���� �ٱ��� �ִ� ���̹Ƿ� NULL �� return �Ͽ� �ش�.
	return NULL;
}

//head �������� �޴´�.
int countNode(Node* start) {

	int count = 0;
	Node* scout = start;

	while (scout != NULL) {
		scout = scout->Next;
		count++;
	}
	return count;
}

//add ����� �������� Ž�������� ������ ���ؼ�
void addNode(Node** start, Node* newNode) {

	//no list exists
	if ((*start) == NULL)  {
		*start = newNode;
	}

	//list exists
	else {
		Node* scout = (*start);
		while (scout->Next != NULL) {
			scout = scout->Next;
		}
		scout->Next = newNode;
		newNode->Prev = scout;
	}
}

//already node serching course is finished
void insertNode(Node* current, Node* newNode) {
	//head
	if(current->Next == NULL &&
	   current->Prev == NULL) {

		// ��ũ�� ����Ʈ�� ���� �Ǵ� �߰� �Ҷ� ������ �߿��ϴ�. 
		// Prev ��� next �� ���� ���� �ϸ� ���� ���簡 �Ǿ� �ùٸ��� �ʰ� �߰� ���� �ʴ� ������ �߻��Ѵ�.
		//current->Next = newNode;
		//newNode->Prev = current;

		newNode->Prev = current;
		current->Next = newNode;
		
	}

	//not head
	
	// if end
	else if (current->Next == NULL) {

		current->Next = newNode;
		newNode->Prev = current;

	}
	
	//in the middle of 2 node
	//if (current->Next != NULL &&
	//	current->Prev != NULL) {
	else{

		Node* next = current->Next;

		next->Prev = newNode;
		newNode->Prev = current;

		current->Next = newNode;
		newNode->Next = next;
	
	}
}

void screenNode(Node* start) {
	while (start != NULL) {
		printf("count : %d\n=>Data : %d\n", countNode(start), start->Data);
		start = start->Next;
	}
}

void removeNode(Node** start, Node* remove) {

	//���� ���� ù��° Node�� ���� �ϰ��� �Ҷ� 
	if (*start == remove) {
		// �ٸ� ��尡 �����ϴ� ��� start�� �ٷ� ������ ������ ������ �Ұ��� �����Ƿ� start�� �������� �Űܳ��� �����Ѵ�.
		*start = remove->Next;
	}

	//if remove has next link
	if (remove->Next != NULL) {
		Node* remove_next = remove->Next;
		remove_next->Prev = remove->Prev;

	}

	//if remove has prev link 
	if (remove->Prev != NULL) {
		Node* remove_prev = remove->Prev;
		remove_prev->Next = remove->Next;
	}
	deleteNode(remove);
}

//�޸� �����ÿ� ������ �ذ�
void clearNode(Node** start_pos) {
	int index = 0;
	Node* start = *start_pos;
	while (start != NULL) {
		//start ��ġ�� �����ϴ´�� getNodeAt���� 
		Node* remove = getNodeAt(start, index);
		//���� �������� �����ϸ� �����ش�.
		//�׸��� index�� �ʱ�ȭ �Ͽ� �ٽ� ������ ������.
		if (remove->Next == NULL) {
			removeNode(&start, remove);
			index = 0;
		}
		else {
			index++;
		}
	}
}

void changeNode(Node** start_pos, Node* current) {

	//if list has only one node or end of the list
	if (current->Next == NULL ) {
		return;
	}

	//current is head
	if (current->Prev == NULL) {

		Node* next = current->Next;
		//change the start pos
		(*start_pos) = current->Next;

		

		//if list has more than 3nodes
		if (next->Next != NULL) {
			next->Next->Prev = current;
		}

		next->Prev = current->Prev;
		current->Prev = next;
		current->Next = next->Next;
		next->Next = current;
		return;
	}

	//current is not head
	//error is here
	else {
		Node* cur_prev = current->Prev;
		Node* cur_next = current->Next;


		if (cur_next->Next != NULL) {
			cur_next->Next->Prev = current;
		}
		cur_next->Prev = current->Prev;
		current->Prev = cur_next;

		current->Next = cur_next->Next;
		cur_next->Next = current;
		cur_prev->Next = cur_next;

		int debug = 0;
		return;
	}
}

//��� �Լ��� �̿��Ͽ� ascendingorder �� ���� �Ͽ����ϴ�.
void ascendingNode(Node** start_pos) {

	//if change_count is 0 return and exit the func
	int count_change = 0;
	Node* scout = *start_pos;

	while (scout->Next != NULL) {
		if (scout->Data > scout->Next->Data) {
			changeNode(start_pos, scout);
			count_change++;
			break;
		}
		scout = scout->Next;
	}

	if (count_change == 0) {
		return;
	}
	else {
		ascendingNode(start_pos);
	}
}

void descendingNode(Node** start_pos) {

	//if change_count is 0 return and exit the func
	int count_change = 0;
	Node* scout = *start_pos;

	while (scout->Next != NULL) {
		if (scout->Data < scout->Next->Data) {
			changeNode(start_pos, scout);
			count_change++;
			break;
		}
		scout = scout->Next;
	}

	if (count_change == 0) {
		return;
	}
	else {
		descendingNode(start_pos);
	}
}

int main(void) {
	Node* start = createNode(0);
	//for (int i = 4; i >= 1; i--) {
	//	addNode(&start, createNode(i));
	//}

	for (int i = 1; i < 5; i++) {
		addNode(&start, createNode(i));
	}

//	screenNode(start);
//	ascendingNode(&start);
	descendingNode(&start);
	screenNode(start);



	clearNode(&start);
	return 0;
}
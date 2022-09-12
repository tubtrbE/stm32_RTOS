#include"LinkedList.h"






Node* createNode(int data) {

	// Node struct 만큼의 공간을 갖는 Node 의 주소를 생성
	Node* newNode = (Node*)malloc(sizeof(Node));

	// 변수 초기화 
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

	//Node 의 주소가 비어 있다면 stop 그렇지 않으면 계속해서 간다.
	while (scout != NULL) {
	
		//만약에 count 와 index 가 같아진다면 정찰병을 return 해줌으로써 함수를 끝낸다.
		if (count++ == index) {
			return scout;
		}
		scout = scout->Next;
	}

	//while loop 를 통과 했음에도 검출하지 못했다면 범위 바깥에 있는 수이므로 NULL 을 return 하여 준다.
	return NULL;
}

//head 시작점을 받는다.
int countNode(Node* start) {

	int count = 0;
	Node* scout = start;

	while (scout != NULL) {
		scout = scout->Next;
		count++;
	}
	return count;
}

//add 노드의 존재이유 탐색과정을 없에기 위해서
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

		// 링크드 리스트를 삽입 또는 추가 할때 순서가 중요하다. 
		// Prev 대신 next 를 먼저 삽입 하면 얇은 복사가 되어 올바르지 않게 추가 되지 않는 현상이 발생한다.
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

	//만약 가장 첫번째 Node를 제거 하고자 할때 
	if (*start == remove) {
		// 다른 노드가 존재하는 경우 start를 바로 제거해 버리면 추적이 불가능 해지므로 start를 다음으로 옮겨놓고 진행한다.
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

//메모리 해제시에 누수를 해결
void clearNode(Node** start_pos) {
	int index = 0;
	Node* start = *start_pos;
	while (start != NULL) {
		//start 위치를 변경하는대신 getNodeAt존재 
		Node* remove = getNodeAt(start, index);
		//만약 끝지점에 도착하면 지워준다.
		//그리고 index를 초기화 하여 다시 루프를 돌린다.
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

//재귀 함수를 이용하여 ascendingorder 을 구현 하였습니다.
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
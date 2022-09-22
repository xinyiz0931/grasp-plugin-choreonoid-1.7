#include <stdio.h>
#include <stdlib.h>
#include "struct.h"


void add_element_to_list(list_data *tl,void *t){
	/* ���_v�̎O�p�`���X�g�ɎO�p�`t�������� */
	
	int flag = 1;

	
	while (tl->next != NULL){
		if (tl->next->element == t){
			flag = 0;
			break;
		}
		tl = tl->next;
	};

	if (flag){
		tl->next = (list_data *)malloc(sizeof(list_data));
		if(tl->next == NULL) fprintf(stderr,"overfwr\n");
		tl->next->element = t;
		tl->next->next = NULL;
	}
}

void add_any_element_to_list(list_data *tl,void *t){
	/* ���_v�̎O�p�`���X�g�ɎO�p�`t�������� */
	
	while (tl->next != NULL){
		tl = tl->next;
	};
	tl->next = (list_data *)malloc(sizeof(list_data));
	if(tl->next == NULL) fprintf(stderr,"overfwr\n");
	tl->next->element = t;
	tl->next->next = NULL;
}

void del_element_from_list(list_data *tl,void *t)
/* ���_v�̎O�p�`���X�g����O�p�`t����菜�� */
{ 
	void *tmp;
	
	while (tl->next != NULL){
		if (tl->next->element == t){
			tmp = tl->next;
			tl->next = tl->next->next;
			free(tmp);
			break;
		}
		tl = tl->next;
	}
}


void *load_list_elements(list_data *tl,int cleartrue){
	static list_data *p, *l;

	if(cleartrue == 1){
		l = p = tl;
		return NULL;
	}
	if(l != tl){
		fprintf(stderr,"Load List elements error , Not cleared list\n");
	}
	if(p->next == NULL) return NULL;
	p = p->next;
	return p->element;
}

void *load_list_element_and_next_pointer(list_data **tl){
	if((*tl)->next == NULL) return NULL;
	*tl = (*tl)->next;
	return (*tl)->element;
}

void delete_list(list_data *tl){ //list���폜����B
	list_data *tmp;

	if(tl == NULL) return;
	
	while (tl->next != NULL){
		tmp = tl;
		tl = tl->next;
		tmp->next = NULL;
		free(tmp);
	}
	free(tl);
}

/*
#ifdef MAKE_CLUSTER_HIERARCHY

ring_list_data*
add_element_to_ring_list(ring_list_data *rl,void *e, int type, object_data *o){
	ring_list_data *tmp;
	static int first=0;

	if(first==0){
		first = 1;
		o->ring_lists = (ring_list_data *)malloc(sizeof(ring_list_data)*o->num_of_ver*2);
		o->Nring_lists = 0;
	}

	if(rl == NULL){
		rl = (ring_list_data *)malloc(sizeof(ring_list_data));
		rl->next = rl;
		rl->previous = rl;
		rl->element = e;
		rl->type = type;
		return rl;
	}
	tmp = rl->next;
	rl->next = (ring_list_data *)malloc(sizeof(ring_list_data));
	rl->next->previous = rl;
	rl->next->next = tmp;
	tmp->previous = rl->next;
	rl->next->element = e;
	rl->next->type = type;

	return rl->next;
}

ring_list_data*
del_list_from_ring_list(ring_list_data *rl,ring_list_data *r){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l == r){
			if(l==l->next){
				free(l);
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
//			free(t);
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



#else
*/

ring_list_data*
add_element_to_ring_list(ring_list_data *rl,void *e, int type){
	ring_list_data *tmp;

	if(rl == NULL){
		rl = (ring_list_data *)malloc(sizeof(ring_list_data));
		rl->next = rl;
		rl->previous = rl;
		rl->element = e;
		rl->type = type;
		return rl;
	}
	tmp = rl->next;
	rl->next = (ring_list_data *)malloc(sizeof(ring_list_data));
	rl->next->previous = rl;
	rl->next->next = tmp;
	tmp->previous = rl->next;
	rl->next->element = e;
	rl->next->type = type;

	return rl->next;
}

ring_list_data*
del_list_from_ring_list(ring_list_data *rl,ring_list_data *r){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l == r){
			if(l==l->next){
#ifndef MAKE_CLUSTER_HIERARCHY
				free(l);
#endif
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
#ifndef MAKE_CLUSTER_HIERARCHY
			free(t);
#endif
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



//#endif

ring_list_data*
del_element_from_ring_list(ring_list_data *rl,void *e){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l->element == e){
			if(l==l->next){
				free(l);
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
			free(t);
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



void print_ring_list(ring_list_data *l){
	ring_list_data *tl;

	if(l==NULL){
		printf("NULL LIST\n");
		return;
	}

	tl = l;
	do{
//		if(l->type == 0)
		if(l->element == NULL) printf("-1,");
		else printf("%d,",((cluster_data *)l->element)->number);
		l= l->next;
	}while(l!=tl);
	printf("\n");

}

int return_number_of_ring_list(ring_list_data *l){
	ring_list_data *tl;
	int count;

	if(l==NULL){
		return 0;
	}
	count = 0;
	tl = l;
	do{
		count++;
		l= l->next;
	}while(l!=tl);
	return count;
}

void downheap(int n, heap_data a[], int k){
	int j;
	heap_data v;
	v = a[k];			// �e�m�[�h�̒l���m��
	while (1) {			// a[k]��e�m�[�h�Ƃ���؂�T��
		j = 2 * k ;		// ���̎q�̃C���f�b�N�Xj���v�Z
		if (j > n) break;		// j��r���z����΂����q�m�[�h���Ȃ��Ɣ��f���C���[�v���яo���D
		if (j != n) {		// j��r��菬�������C
			if (a[j + 1].quadrics > a[j].quadrics) {	// a[j+1]��a[j]���傫���Ƃ��ɂ́C
				j = j + 1;		// j����i�߂�D
			}
		}
		if (v.quadrics >= a[j].quadrics) break;	// �e�̕����傫���ꍇ�ɂ̓��[�v���яo���D
		a[k] = a[j];		// �����ɓ��B����Ƃ��ɂ́C
		// �q�m�[�h���e�m�[�ha[k]���傫���̂�a[j]�̒l������D
		k = j;			// ���̎q�m�[�h��V���Ȑe�m�[�h�ɂ��āC�q�[�v���\���������D
	}
	a[k] = v;			// ���̒l��召�֌W��ۂm�[�h�ɓ����D
}

// ���ڂ��Ă���v�f��K�؂ȏꏊ�܂ŕ��������点��
void upheap(int n, heap_data a[], int k) //n���� a�z�� ���ڔԍ�
{
    heap_data x;
	
    x = a[k];
	while (k > 1 && a[k/2].quadrics < x.quadrics) {
		a[k] = a[k/2];
		k /= 2;
	}
	a[k] = x;
}


// "Copyright [2016] <Leonardo Passig Horstmann>"
#include "Elemento.hpp"
#include <cstdlib>
#define ERROLISTAVAZIA -1
#define ERROLISTACHEIA -2
#define ERRODEPOSICAO -3
#define ERRONAOENCONTRADO -4
#ifndef LISTAENC_HPP
#define LISTAENC_HPP
template<typename T>
class ListaEnc {
public:
	Elemento<T>* head;  //!< ponteiro que aponta pro inicio da lista
	int size;  //!< tamanho inteiro da lista


    //! metodo que altera valor de head
    /*!
     *Atribui ponteiro passado por parametro para head
     \param nHead ponteiro para elemento
     */
    void setHead(Elemento<T> *nHead) {
        head = nHead;
    }
    //! metodo que altera size
    /*!
     *Atribui o valor passado como parametro para size
     \param nSize inteiro que representa novo tamanho
     */
    void setSize(int nSize) {
        size = nSize;
    }
    //! metodo que retorna head
    /*!
     *retorna o ponteiro que aponta para o inicio da lista
     \return ponteiro para elemento
     */
    Elemento<T>* getHead() {
        return head;
    }
    //! retorna tamanho
    /*!
     *retorna o tamanho da lista
     \return inteiro tamanho
     */
    int getSize() {
        return size;
    }

 public:
    //! Construtor
    /*!
     *Atribui o tamanho -1, e o ponteiro vira NULL (valores default)
     */
	ListaEnc() {
	    size = -1;
	    head = NULL;
	}
	//! Destrutor
	/*!
	 *passa a lista eliminando os dados depois coloca os valores de head e size 
	 *para o default
	 */
	~ListaEnc() {
	    if (!listaVazia()) {
    	    int i;
    	    for (i = 0; i < size; i++) {
    	        eliminaDoInicio();
    	    }
    	 }
    	 destroiLista();
	}
	// inicio
	//! adiciona elemento no inicio da lista
	/*!
	 *adiciona o dado passado como parametro na primeira posicao da lista.
	 *verifica se e possivel criar um novo elemento, aponta o campo endereco pro
	 *elemento anteriormente apontado por head e faz com que head aponte pra ele
	 *entao incrementa size
	 \param dado endereco constante para um dado do tipo T
	 */
	void adicionaNoInicio(const T& dado) {
	    Elemento<T> *novo;
	    novo = new Elemento<T>(dado, head);
	    if (novo == NULL) {
	        throw(ERROLISTACHEIA);
	    } else {
	        head = novo;
	        size++;
	    }
	}
	//! metodo que retira do inicio
	/*!
	 *retira o primeiro elemento da lista (se ela nao estiver vazia), retorna o
	 *dado contido neste elemento e exclui seu ponteiro
	 \return dado do tipo T
	 */
	T retiraDoInicio() {
	    Elemento<T> *saiu;
	    T *volta = new T;
	    if (listaVazia()) {
	        throw(ERROLISTAVAZIA);
	    } else {
	        saiu = head;
	        *volta = saiu->getInfo();
	        head = saiu->getProximo();
	        size--;
	        delete saiu;
	        return (*volta);
	    }
	}
	//! elimina um elemento do inicio da fila
	/*!
	 *faz o mesmo procedimento do metodo retira do inicio, porem sem retorno
	 \sa retiraDoInicio()
	 */
	void eliminaDoInicio() {
	    Elemento<T> *saiu;
	    if (listaVazia()) {
	        throw(ERROLISTAVAZIA);
	    } else {
	        saiu = head;
    	    head = saiu->getProximo();
    	    // delete saiu;
	    }
	}
	// posicao
	//! adiciona dado passado na posicao recebida
	/*!
	 *adiciona o dado passado na posicao a frente da posicao do dado anteior -
	 *passada via ponteiro, se for possivel criar um novo elemento, havendo 
	 *sucesso, incrementa size.
	 \param dado Endereco para um dado do tipo T
	 \param pos ponteiro para um elemento
	 */
	void adicionaNaPosicao(const T& dado, Elemento<T> *pos) {
	    Elemento<T> *novo = new Elemento<T>(dado, pos->getProximo());
        if (&novo == NULL) {
            throw(ERROLISTACHEIA);
        }
        pos->setProximo(novo);
        size++;
	}
	//! adiciona um dado na posicao passada
	/*!
	 *adiciona o dado passado na posicao passada, primeiramente verifica o valor
	 *da posicao, para determinar se ela é ou nao valida, alem disso, verifica
	 *tambem a possibilidade de criar um novo elemento na memoria, apos processo
	 *chama a versao com ponteiro do metodo
	 \param dado endereco para um dado do tipo T
	 \param pos inteiro que representa a posicao que o dado deve ser inserido
	 */
	void adicionaNaPosicao(const T& dado, int pos) {
    	if ((pos >= 1) && (pos < size+1)) {
    	    Elemento<T>* aux = head;
            int i= 0;
            while (i < pos-1) {
                aux = aux->getProximo();
                i++;
            }
            Elemento<T>* novo = new Elemento<T>(dado, aux->getProximo());
            if (&novo == NULL) {
                throw(ERROLISTACHEIA);
            }
            aux->setProximo(novo);
            size++;
        } else {
            if (pos == 0) {
                adicionaNoInicio(dado);
            } else if (pos == size+1) {
                adiciona(dado);
            } else {
                throw(ERRODEPOSICAO);
            }
        }
	}
	//! retorna a posicao de um dado na lista
	/*!
	 *retorna a posicao em inteiro de um dado na lista, se a lista nao estiver
	 *vazia e o dado existir na mesma.
	 \param dado endereco constante de um dado do tipo T
	 \return inteiro que representa a posicao do dado
	 */
	int posicao(const T& dado) const {
        if (!listaVazia()) {
	        Elemento<T>* aux = head;
            int i= 0;
            for (i = 0; i <= size; i++) {
                if (aux->getInfo() == dado) {
                    return i;
                }
                aux = aux->getProximo();
            }
            throw(ERRONAOENCONTRADO);
        } else {
            throw(ERROLISTAVAZIA);
        }
	}
	//! retorna a posicao de um elemento na memoria
	/*!
	 *retorna o endereco de um elemento na memoria, se, o mesmo estiver contido na 
	 *lista
	 \param dado endereco constante de um dado do tipo T
	 \return ponteiro para um elemento
	 */
	Elemento<T>* posicaoElementoMem(const T& dado) const {
        if (!listaVazia()) {
	        Elemento<T>* aux;
            int i= 0;
            for (i = 0; i <= size; i++) {
                if (i == 0) {
                    aux = head;
                } else {
                    aux = aux->getProximo();
                }
                if (aux->getInfo() == dado) {
                    return aux;
                }
            }
            throw(ERRONAOENCONTRADO);
        } else {
            throw(ERROLISTAVAZIA);
        }
	}
	//! retorna a posicao de um dado na memoria
	/*!
	 *retorna o endereco de um dado na memoria, se, o mesmo estiver contido na 
	 *lista
	 \param dado endereco constante de um dado do tipo T
	 \return ponteiro para um dado do tipo T
	 */
	T* posicaoMem(const T& dado) const {
        if (!listaVazia()) {
	        Elemento<T>* aux;
            int i= 0;
            for (i = 0; i <= size; i++) {
                if (i == 0) {
                    aux = head;
                } else {
                    aux = aux->getProximo();
                }
                if (aux->getInfo() == dado) {
                    return aux;
                }
            }
            throw(ERRONAOENCONTRADO);
        } else {
            throw(ERROLISTAVAZIA);
        }
	}
	//! informa se um dado existe na lista
	/*!
	 *verifica se um dado esta contido na lista (posicao >= 0) e retorna true ou
	 *false
	 \param dado endereco constante para um dado do tipo T
	 \return boolean true or false
	 */
	bool contem(const T& dado) {
	    try {
	        return (posicao(dado) >= 0);
	    } catch(...) {
	        return false;
	    }
	}
	//! retira um elemento de uma posicao e retorna o dado nele contido
	/*!
	 *retira um dado da posicao passada, se o dado existir na lista.
	 \param pos inteiro que representa a posicao do dado a ser retirado
	 \return dado do tipo T
	 */
	T retiraDaPosicao(int pos) {
	    if (listaVazia()) {
	        throw(ERROLISTAVAZIA);
	    } else {
	        if ((pos > 0) && (pos <= size)) {
        	    Elemento<T> *saiu = head;
        	    Elemento<T> *aux;
        	    T *volta = new T;
        	    int i;
        	    for (i = 0; i < pos; i++) {
                    aux = saiu;
        	        saiu = saiu->getProximo();
        	    }
	            *volta = saiu->getInfo();
    	        aux->setProximo(saiu->getProximo());
    	        size--;
    	        delete saiu;
    	        return (*volta);
	        } else {
	            if (pos == 0) {
	                return (retiraDoInicio());
	            } else {
	                throw(ERRODEPOSICAO);
	            }
	        }
	    }
	}
	//! adiciona elemento no fim da fila com o dado do parametro
	/*!
	 *adiciona
	 *adiciona um dado no final da lista. Para isso percorre a lista inteira, se
	 *nao for possivel criar novo elemento, lanca excessao, se o processo tiver
	 *sucesso, incrementa size
	 \param dado endereco constante de um dado do tipo T
	 */
	void adiciona(const T& dado) {
	    if (listaVazia()) {
	        adicionaNoInicio(dado);
	    } else {
	        Elemento<T> *aux = head;
    	    Elemento<T> *novo;
    	    novo = new Elemento<T>(dado, NULL);
    	    if (novo == NULL) {
    	        throw(ERROLISTACHEIA);
    	    } else {
	            int i;
    	        for (i = 0; i < size; i++) {
    	            if (aux->getProximo() != NULL) {
    	                aux = aux->getProximo();
    	            }
    	        }
    	        aux->setProximo(novo);
    	        size++;
        	}
        }
    }
    //! retira elemento do fim da lista e retorna dado contido
    /*!
     *retira e retorna o dado da ultima posicao da lista
     \return dado do tipo T
     */
	T retira() {
	     return (retiraDaPosicao(size));
	}
	// especifico
	//! retira elemento que contem dado especifico
	/*!
	 *retira um dado especifico da lista, se a lista nao estiver vazia e se o
	 *dado estiver contido na lista
	 \param dado endereco constante para um dado do tipo T
	 \return dado do tipo T
	 */
	T retiraEspecifico(const T& dado) {
	    if (contem(dado)) {
	        return(retiraDaPosicao(posicao(dado)));
	    } else {
	        throw(ERRONAOENCONTRADO);
	    }
	}
	//! adiciona dado em ordem
	/*!
	 *adiciona o dado passado em ordem crescente, apenas verifica ate achar um
	 *dado na lista com valor superior, tendo localizado passa o ponteiro deste
	 *dado para o metodo adicionaNaPosicao
	 \param data endereco constante de um dado do tipo T
	 */
	void adicionaEmOrdem(const T& data) {
	    if (!listaVazia()) {
	        Elemento<T>* aux = head;
	        int i = 0;
	        while ((aux->getProximo()!=NULL) && maior(data, aux->getInfo())) {
	            i++;
	            aux = aux->getProximo();
	        }
	        if (maior(data, aux->getInfo())) {
				adicionaNaPosicao(data, i+1);
			} else {
				adicionaNaPosicao(data, i);
			}
	    } else {
	        adicionaNoInicio(data);
	    }
	}
	//! mostra um dado em uma posicao
	/*!
	 *retorna o dado presente no elemento da posicao "pos" da lista
	 \param pos inteiro que representa a posicao
	 \return dado do tipo T
	 */
    T mostra(int pos) {
	    if (listaVazia()) {
	        throw(ERROLISTAVAZIA);
	    } else {
	        if (pos >= 0 && pos <= size) {
    	        Elemento<T> *aux = head;
    	        int i;
    	        for (i = 0; i < pos; i++) {
    	            aux = aux->getProximo();
    	        }
    	        return ((aux->getInfo()));
    	    }
    	    throw(ERRODEPOSICAO);
	    }
	}
	//! mostra um dado em uma posicao
	/*!
	 *retorna o dado presente no elemento apontado pelo parametro pos
	 \param pos ponteiro para um elemento que contem o dado a ser mostrado
	 \return dado do tipo T
	 */
	T mostra(Elemento<T>* pos) {
	    if (pos != NULL) {
	        return (pos->getInfo());
	    } else {
	        throw(ERRODEPOSICAO);
	    }
	}
	//! define se lista esta vazia
	/*!
	 *retorna o resultado da comparacao ultimo = -1, se verdadeiro, lista vazia
	 \return booleano
	 */
	bool listaVazia() const {
	    return (size == -1);
	}
	//! metodo que define se dois dados sao iguais
	/*!
	 *retorna o resultado da comparacao (igualdade) entre os parametros dado1 e 
	 *dado2, se forem iguais, retorno verdadeiro, se nao falso.
	 \param dado1 dado do tipo T
	 \param dado2 dado do tipo T
	 \return booleano
	 */
	bool igual(T dado1, T dado2) {
	    return (dado1 == dado2);
	}
	//! metodo que define se dado1 > dado2
	/*!
	 *retorna o resultado da comparacao (maior que) entre os parametros dado1 e 
	 *dado2, se dado1 > dado2, retorno verdadeiro, se nao falso.
	 \param dado1 dado do tipo T
	 \param dado2 dado do tipo T
	 \return booleano
	 */
	bool maior(T dado1, T dado2) {
	    return (dado1 > dado2);
	}
	//! metodo que define se dado1 < dado2
	/*!
	 *retorna o resultado da comparacao (menor que)entre os parametros dado1 e 
	 *dado2, se forem iguais, retorno verdadeiro, se nao falso.
	 \param dado1 dado do tipo T
	 \param dado2 dado do tipo T
	 \return booleano
	 */
	bool menor(T dado1, T dado2) {
	    return (dado1 < dado2);
	}
	//! metodo que atribui valores default as variaveis
	/*!
	 *seta os valores de size e head para o default
	 */
	void destroiLista() {
	    size = -1;
	    head = NULL;
	}
};

#endif

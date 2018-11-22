Mudanças para a versão com heurística:

include/scheduler.h:
	- Criada a variável "heurístic", que representa se a execução
	vai fazer uso da heurística.

include/thread;h:
	- Criado o vetor "_clock_factor", utilizado pela heurística para
	controlar o duty cycle aplicado na modulação de clock.
	- Criado o vetor "_slowdown", utilizada pela heurística para
	controlar a variação do fator quando o mesmo é incrementado.
	- Criados os vetores "_bef_channel6" e "_bef_channel5" utilizados
	pela heurística para armazenar o crescimento anterior do canal
	a fim de auxiliar a tomar decisões sobre o _clock_factor.

include/pmu.h:
	- Alterado o valor do "_channel_3" para 14.
	- Alterado o valor do "_channel_5" para 138.
	- Alterado o valor do "_channel_6" para 139.
	(Sendo estes os eventos utilizados pela heurística)

include/periodic_thread.h:
	- wait_next():
		- Adição do código da heurística.

src/component/thread.cc:
	- dispatch():
		- Adição do código da heurística.
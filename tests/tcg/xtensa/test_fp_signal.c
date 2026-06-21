#include <signal.h>
#include <stdlib.h>
#include <sys/ucontext.h>

/* This level of optimization produces code that uses
 * the `lsi` opcode that implements the `a = *p` and
 * it's skipped by the `sc_pc += 3` in the handle_signal()
 */
#pragma GCC optimize ("Os")

int test_fp_signal(int n, int s, float start, float *p)
{
	float a = start;
	float b = 0.f;
	int i;

	for (i = 0; i < n; ++i) {
		b += a;
		if (i == s)
			a = *p;
	}
	return b;
}

int tmp;

static void handle_signal(int sig_no, siginfo_t* info, void *vcontext)
{
	ucontext_t *context = (ucontext_t *)vcontext;

	context->uc_mcontext.sc_pc += 3;
	tmp = test_fp_signal(5, 6, 2.f, NULL);
}

int main()
{
	struct sigaction sig_action = {
		.sa_sigaction = handle_signal,
		.sa_flags = SA_RESTART | SA_SIGINFO,
	};
	sigaction(SIGSEGV, &sig_action, 0);
	if (test_fp_signal(5, 1, 1.f, NULL) != 5)
		abort();
	return 0;
}

/*------------------------------------------------------------------*/
/// @file		main.cpp
/// @brief		Viewer main
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <csignal>
#include <cstdlib>
#include <vector>
#include <memory>

#include "ViewerGui.h"
#include "CommonLog.h"

static std::unique_ptr<krm::ViewerGui> gui = nullptr;


static void killHandler(int sig)
{
	LOG_ERR("Abort(signal:%d)\n", sig);
	gui.reset(nullptr);
	std::exit(1);
}

int main(int argc, char* argv[])
{
	std::vector<int> signals = {SIGABRT, SIGFPE, SIGILL, SIGINT, SIGSEGV, SIGTERM};

	(void)argc;
	(void)argv;

	/* add signal handlers for failsafe */
	for (auto sig : signals) {
		if (SIG_ERR == std::signal(sig, killHandler)) {
			LOG_WRN("Signal(%d) : errno = %d\n", sig, errno);
		}
	}

	gui = std::make_unique<krm::ViewerGui>();
	if (gui != nullptr) {
		gui->run();
	}

	return 0;
}

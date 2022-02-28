#include <iostream>
#include <bitset>

#include "System.h"

int main(int argc, char* argv) {
	System sys = System();
	sys.OpenRom("eds.a26");

	sys.Init();

	while (true) {
		sys.DecodeExecuteOP();

		std::cin.get();
	}

	return 0;
}
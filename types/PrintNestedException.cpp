#include "PrintNestedException.h"
#include <iostream>
#include <stdexcept>
#include <string>

using namespace SF;

void SF::print_exception(const std::exception & e, int level)
{
	std::cerr << std::string(level, ' ') << "exception: " << e.what() << '\n';
	try {
		std::rethrow_if_nested(e);
	}
	catch (const std::exception& e) {
		print_exception(e, level + 1);
	}
	catch (...) {}
}

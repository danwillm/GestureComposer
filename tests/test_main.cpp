#include "test_framework.h"

int main(int argc, char** argv) { return gctest::runAll(argc > 1 ? argv[1] : nullptr); }

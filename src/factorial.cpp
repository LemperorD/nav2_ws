#include "factorial.hpp"

long long calculateFactorial(int number) {
    long long result = 1;
    for (int i = 1; i <= number; ++i) {
        result *= i;
    }
    return result;
}

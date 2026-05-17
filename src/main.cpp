#include <iostream>
#include "factorial.hpp"

int main() {
    int userInput;
    std::cout << "请输入一个整数: ";
    std::cin >> userInput;

    long long result = calculateFactorial(userInput);
    std::cout << userInput << " 的阶乘是: " << result << std::endl;

    return 0;
}

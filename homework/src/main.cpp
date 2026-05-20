#include "factorial.h"
#include <iostream>

int main()
{
    int n;
    std::cout << "输入一个整数: ";
    std::cin >> n;
    std::cout << n <<" 的阶乘是： " << calculate(n) << std::endl;
    return 0;
}

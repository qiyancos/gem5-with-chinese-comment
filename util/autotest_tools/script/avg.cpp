#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>

#define USE_ARITHMETIC_AVG 1
#define USE_GEOMEAN 0

int main() {
    bool isFloat = false;
    std::string numberStr;
    int count, total;
    std::cin >> count;
    total = count;
    int realCount = 0;
#if (USE_ARITHMETIC_AVG == 1)
    double sum = 0.0;
#else
    double sum = 1.0;
#endif
    while (count--) {
        std::cin >> numberStr;
        if (numberStr.find('.') != std::string::npos) {
            isFloat = true;
        }
        if (numberStr[0] >= '0' && numberStr[0] <= '9') {
#if (USE_ARITHMETIC_AVG == 1)
            sum += atof(numberStr.c_str());
#else
            sum *= atof(numberStr.c_str());
#endif
            realCount++;
        }
    }
#if (USE_ARITHMETIC_AVG == 1)
    double resultFloat = sum / total;
    int resultInt = sum / total;
#else
    double resultFloat = std::pow(sum, 1.0 / total);
    int resultInt = std::pow(sum, 1.0 / total);
#endif
    if (!realCount) {
        std::cout << "nan" << std::endl;
    } else if (isFloat) {
        std::cout << resultFloat << std::endl;
    } else {
        std::cout << resultInt << std::endl;
    }
    return 0;
}

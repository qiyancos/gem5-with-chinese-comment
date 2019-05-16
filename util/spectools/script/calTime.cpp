#include <iostream>
#include <vector>

int main(){
    std::vector<float> time;
    float temp;
    while(std::cin >> temp)
        time.push_back(temp);
    unsigned long long total;
    if(time.size() == 3) total = time[0] * 3600000 + 
            time[1] * 60000 + time[2] * 1000;
    else total = time[0] * 60000 + time[1] * 1000;
    std::cout << total << std::endl;
}

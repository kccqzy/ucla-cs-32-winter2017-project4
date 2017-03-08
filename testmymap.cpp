#include <string>
#include "MyMap.h"
#include <cassert>
#include <cstdio>
#include <map>
#include <unordered_map>

#define printsize(...) printf("sizeof(%s) = %zu\n", #__VA_ARGS__, sizeof(__VA_ARGS__))

int main() {
    printsize(MyMap<std::string, double>);
    printsize(std::map<std::string, double>);
    printsize(std::unordered_map<std::string, double>);
    MyMap<std::string, double> nameToGPA;
    nameToGPA.associate("Carey", 3.50);
    nameToGPA.associate("David", 3.99);
    nameToGPA.associate("Abe", 3.20);
    assert(nameToGPA.size() == 3);
    double* davidGPA = nameToGPA.find("David");
    assert(davidGPA);
    *davidGPA = 1.5;
    nameToGPA.associate("Carey", 4.0);
    assert(*nameToGPA.find("David") == 1.5);
    assert(*nameToGPA.find("Carey") == 4.0);
    assert(!nameToGPA.find("Linda"));
    assert(nameToGPA.size() == 3);
}

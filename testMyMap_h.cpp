#include "MyMap.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

void nachenbergTest() {
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

void permutations_in_order() {
    MyMap<std::string, size_t> permutation;

    size_t const n = 9;
    size_t const nfactorial = 362880;
    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        permutation.associate(cur, 0);
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            permutation.associate(cur, i);
        }
    }

    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            size_t* answer = permutation.find(cur);
            assert(answer);
            assert(*answer == i);
        }
    }
}

void permutations_in_order_std_map() {
    std::map<std::string, size_t> permutation;

    size_t const n = 9;
    size_t const nfactorial = 362880;
    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        permutation.emplace(cur, 0);
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            permutation.emplace(cur, i);
        }
    }

    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            auto answer = permutation.find(cur);
            assert(answer != permutation.end());
            assert(answer->second == i);
        }
    }
}

void permutations_random() {
    MyMap<std::string, size_t> permutation;
    std::vector<std::pair<std::string, size_t>> v;

    size_t const n = 9;
    size_t const nfactorial = 362880;
    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        v.emplace_back(cur, 0);
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            v.emplace_back(cur, i);
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(v.begin(), v.end(), g);
    for (auto const& s : v) permutation.associate(s.first, s.second);

    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            size_t* answer = permutation.find(cur);
            assert(answer);
            assert(*answer == i);
        }
    }
}

void permutations_random_std_map() {
    std::map<std::string, size_t> permutation;
    std::vector<std::pair<std::string, size_t>> v;

    size_t const n = 9;
    size_t const nfactorial = 362880;
    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        v.emplace_back(cur, 0);
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            v.emplace_back(cur, i);
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(v.begin(), v.end(), g);
    for (auto const& s : v) permutation.emplace(s.first, s.second);

    {
        std::string cur;
        for (size_t i = 0; i < n; ++i) cur += (char) (i + 'a');
        for (size_t i = 1; i < nfactorial; ++i) {
            std::next_permutation(cur.begin(), cur.end());
            auto answer = permutation.find(cur);
            assert(answer != permutation.end());
            assert(answer->second == i);
        }
    }
}


class Timer {
public:
    Timer() { start(); }
    void start() { m_time = std::chrono::steady_clock::now(); }
    double elapsed() const {
        std::chrono::duration<double, std::milli> diff = std::chrono::steady_clock::now() - m_time;
        return diff.count();
    }

private:
    std::chrono::steady_clock::time_point m_time;
};

int main() {
    nachenbergTest();

    Timer timer;
    permutations_in_order();
    std::cout << "Permutations in order: MyMap: " << timer.elapsed() << " milliseconds\n";

    timer.start();
    permutations_in_order_std_map();
    std::cout << "Permutations in order: std::map: " << timer.elapsed() << " milliseconds\n";

    timer.start();
    permutations_random();
    std::cout << "Permutations random: MyMap: " << timer.elapsed() << " milliseconds\n";

    timer.start();
    permutations_random_std_map();
    std::cout << "Permutations random: std::map: " << timer.elapsed() << " milliseconds\n";
}

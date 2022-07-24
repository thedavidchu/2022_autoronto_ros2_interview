#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

std::string ivec2str(const std::vector<std::int8_t> &input) {
    std::stringstream ss;

    ss << "[";
    for (auto &i : input) {
        ss << static_cast<int>(i) << ",";
    }

    ss << "]";
    return ss.str();
}


void test_helper_ivec2str(const std::vector<std::int8_t> &input,
        const std::string &oracle) {
    std::string output = ivec2str(input);
    if (oracle != output) {
        std::cout << "Oracle: " << oracle << " | Output: " << output << std::endl;
        assert(0 && "wrong answer");
    }
}

void test_ivec2str() {
    std::vector<std::int8_t> a = {1, 2, 3, 4, 5, 6};
    test_helper_ivec2str(a, "[1,2,3,4,5,6,]");

    std::vector<std::int8_t> b = {};
    test_helper_ivec2str(b, "[]");

    std::vector<std::int8_t> c = {0, 0, -1, -1, 2, 2};
    test_helper_ivec2str(c, "[0,0,-1,-1,2,2,]");

    std::vector<std::int8_t> d = {1};
    test_helper_ivec2str(d, "[1,]");

    std::vector<std::int8_t> e = {127, -128};
    test_helper_ivec2str(e, "[127,-128,]");
}

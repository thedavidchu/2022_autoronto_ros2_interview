#pragma once

#include <cassert>  // For assert
#include <cstddef>  // For std::int8_t
#include <cstdint>  // For std::int8_t, INT8_MAX
#include <iostream>
#include <vector>
#include <unordered_map>

std::vector<std::int8_t> find_indices(
        const std::vector<std::int8_t> &input, const std::int8_t target) {
    /// Find two unique indices of numbers in $input whose sum is $target.
    ///
    /// Assumptions:
    /// 1. There exists a solution.
    /// 2. The solution is unique.
    /// 3. The order of the returned indices does not matter.
    ///
    /// References:
    /// http://leetcode.com/problems/two-sum/
    /// 
    /// Thoughts:
    /// 1. We return the indices through the same topic that the $inputs come
    /// on, so presumably these are int8_t too.
    /// 2. Since a unique solution exists, there cannot be duplicates of either
    /// number we are looking for. This means that the number of elements is
    /// theoretically unbounded.
    /// E.g. input = [1 ... 1 2 3]
    ///      target = 5
    std::unordered_map<std::int8_t, const std::int8_t> seen_nums;

    assert(input.size() <= INT8_MAX &&
            "we do not have enough storage space unless we do some sketchy "
            "casts from signed to unsigned (and that only buys us a little bit "
            "of space)");
    for (std::int8_t current_idx = 0; current_idx < static_cast<std::int8_t>(input.size());
            ++current_idx) {
        const std::int8_t current_val = input[current_idx];
        int stored_val = target - current_val;

        if (seen_nums.count(stored_val)) {
            const std::int8_t stored_idx = seen_nums[stored_val];
            return {stored_idx, current_idx};
        }
        seen_nums.insert({current_val, current_idx});
    }
    assert(0 && "impossible! Solution not found!");
}

void test_case_find_indices_input(const std::vector<std::int8_t> &input,
        const std::int8_t target, const std::vector<std::int8_t> oracle) {
    std::vector<std::int8_t> output = find_indices(input, target);
    assert(output.size() == 2 && "output is the wrong size!");
    assert(oracle.size() == 2 && "oracle is the wrong size!");

    if (!(oracle[0] == output[0] && oracle[1] == output[1]) &&
            !(oracle[0] == output[1] && oracle[1] == output[0])) {
        std::cout << "Oracle: " << static_cast<std::int16_t>(oracle[0]) <<
                ", " << static_cast<std::int16_t>(oracle[1]) << " | Output: " <<
                static_cast<std::int16_t>(output[0]) << ", " <<
                static_cast<std::int16_t>(output[1]) << std::endl;
        assert(0 && "wrong answer");
    }
}


void test_find_indices() {
    std::vector<std::int8_t> a = {1, 1, 1, 1, 1, 3, 5};
    test_case_find_indices_input(a, 8, {5, 6});
    
    std::vector<std::int8_t> b = {1, 2, 3, 4, 5, 6, 7, 8, 6, 5, 4, 3, 2, 1};
    test_case_find_indices_input(b, 15, {6, 7});

    std::vector<std::int8_t> c = {1, 2, 3, 4, 1, 2, 3, 4, 9, 10}; // Test repeat 
    test_case_find_indices_input(c, 8, {3, 7});
}

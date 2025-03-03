// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.math;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.SortedSet;

/**
 * A helper class providing combinatorics methods to Java collections
 */
public final class Combinatorics {
    private Combinatorics() {}

    /**
     * returns all possible permutations of length 2 of the elements of the set
     */
    public static <T> Set<List<T>> permuteTwo(Set<T> set) {
        Set<List<T>> permutationSet = new HashSet<>();
        for (T elem : set) {
            for (T other : set) {
                if (elem.equals(other)) {
                    continue;
                }
                var permutation = new ArrayList<T>(2);
                permutation.add(elem);
                permutation.add(other);
                permutationSet.add(permutation);
            }
        }
        return permutationSet;
    }

    /**
     * returns all possible combinations of length 2 of the elements of the set
     */
    public static <T> Set<Set<T>> chooseTwo(Set<T> set) {
        Set<Set<T>> combinationSet = new HashSet<>();
        for (T elem : set) {
            for (T other : set) {
                if (elem.equals(other)) {
                    continue;
                }
                var combination = new HashSet<T>(2);
                combination.add(elem);
                combination.add(other);
                combinationSet.add(combination);
            }
        }
        return combinationSet;
    }


    /**
     * N permute K
     */
    public static long nPk(int N, int K) {
        return MathUtils.Factorial(N)/MathUtils.Factorial(N - K);
    }

    /**
     * N choose K
     */
    public static long nCk(int N, int K) {
        return nPk(N, K)/MathUtils.Factorial(K);
    }
}

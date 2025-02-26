// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.functional;

import java.util.Objects;
import java.util.function.Function;

@FunctionalInterface
public interface TriFunction<T, U, V, R> {

    public abstract R apply(T t, U u, V v);

    /** Composes this trifunction with another function */
    public default <W> TriFunction<T, U, V, W> andThen(Function<? super R, ? extends W> after) {
        Objects.requireNonNull(after);
        return (final T t, final U u, final V v) -> after.apply(this.apply(t, u, v));
    }
}

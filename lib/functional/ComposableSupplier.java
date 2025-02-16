// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.functional;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

@FunctionalInterface
public interface ComposableSupplier<T> extends Supplier<T> {
    public default <R> ComposableSupplier<R> andThen(Function<? super T,? extends R> after) {
        Objects.requireNonNull(after);
        return () -> after.apply(this.get());
    }
}

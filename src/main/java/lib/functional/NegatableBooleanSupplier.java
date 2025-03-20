// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.functional;

import java.util.function.BooleanSupplier;

public interface NegatableBooleanSupplier extends BooleanSupplier {
    public default NegatableBooleanSupplier negate() {
        return () -> !this.getAsBoolean();
    }
}

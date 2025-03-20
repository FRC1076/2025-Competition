package lib.functional;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;

public final class FunctionalUtils {
    // Debounces a booleanSupplier
    public static BooleanSupplier debounce(BooleanSupplier supplier,double debounceTime,Debouncer.DebounceType type){
        return new BooleanSupplier() {
            private final Debouncer debouncer = new Debouncer(debounceTime,type);
            private final BooleanSupplier rawSupplier = supplier;

            @Override
            public boolean getAsBoolean() {
                return debouncer.calculate(rawSupplier.getAsBoolean());
            }
        };
    }

    public static BooleanSupplier debounce(BooleanSupplier supplier,double debounceTime){
        return debounce(supplier,debounceTime,Debouncer.DebounceType.kRising);
    }
}

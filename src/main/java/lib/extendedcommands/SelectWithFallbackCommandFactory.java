// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SelectWithFallbackCommandFactory<K> {
    private final Map<K,Supplier<Command>> commandSupplierMap;
    private final Supplier<Command> defaultCommandSupplier;
    private final Supplier<K> selector;

    public SelectWithFallbackCommandFactory(Map<K,Supplier<Command>> commandSupplierMap,Supplier<Command> defaultCommandSupplier, Supplier<K> selector) {
        this.commandSupplierMap = commandSupplierMap;
        this.defaultCommandSupplier = defaultCommandSupplier;
        this.selector = selector;
    }

    private Map<K,Command> buildCommandMap() {
        Map<K,Command> commandMap = new HashMap<>();
        for (Entry<K,Supplier<Command>> entry : commandSupplierMap.entrySet()) {
            commandMap.put(entry.getKey(),entry.getValue().get());
        }
        return commandMap;
    }

    public Command buildCommand() {
        return new SelectWithFallbackCommand<>(buildCommandMap(),defaultCommandSupplier.get(),selector);
    }
}

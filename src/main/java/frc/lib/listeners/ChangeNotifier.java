package frc.lib.listeners;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ChangeNotifier<T> extends BaseChangeNotifier {
    private final Supplier<T> m_supplier;
    private final List<Consumer<T>> m_consumers;
    private T m_currentValue;
    private T m_previousValue;

    protected ChangeNotifier(Supplier<T> value) {
        m_supplier = value;
        m_consumers = new ArrayList<>();

        registerChangeNotifier(this);
    }

    @Override
    protected void updateInternalState() {
        m_previousValue = m_currentValue;
        m_currentValue = m_supplier.get();
    }

    @Override
    protected boolean shouldNotify() {
        return m_currentValue != null && !m_currentValue.equals(m_previousValue);
    }

    @Override
    protected void notifyConsumers() {
        m_consumers.forEach(x -> x.accept(m_currentValue));
    }

    public ChangeNotifier<T> addListener(Consumer<T> onUpdated) {
        m_consumers.add(onUpdated);
        return this;
    }

    public static <T> ChangeNotifier<T> of(Supplier<T> value) {
        return new ChangeNotifier<>(value);
    }

    public static <T> ChangeNotifier<T> ofLogged(String key, Supplier<T> value) {
        return new LoggedChangeNotifier<>(key, value);
    }

    public T getValue() {
        return m_currentValue;
    }
}

package net.bytebuddy.android;

import net.bytebuddy.description.type.TypeDescription;
import net.bytebuddy.dynamic.loading.ClassLoadingStrategy;

import java.io.File;
import java.util.Map;

public class AndroidClassLoadingStrategy implements ClassLoadingStrategy<ClassLoader> {
    @Override
    public Map<TypeDescription, Class<?>> load(ClassLoader classLoader, Map<TypeDescription, byte[]> types) {
        return null;
    }

    public static class Wrapping extends AndroidClassLoadingStrategy {

        public Wrapping(File privateDirectory) {
        }
    }
}
